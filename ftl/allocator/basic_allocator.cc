// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "ftl/allocator/basic_allocator.hh"

#include "ftl/mapping/abstract_mapping.hh"

namespace SimpleSSD::FTL::BlockAllocator {

BasicAllocator::BasicAllocator(ObjectData &o, Mapping::AbstractMapping *m)
    : AbstractAllocator(o, m),
      inUseBlockMap(nullptr),
      freeBlocks(nullptr),
      mtengine(rd()) {
  selectionMode = (Config::VictimSelectionMode)readConfigUint(
      Section::FlashTranslation, Config::Key::VictimSelectionPolicy);
  countMode = (Config::GCBlockReclaimMode)readConfigUint(
      Section::FlashTranslation, Config::Key::GCMode);

  switch (countMode) {
    case Config::GCBlockReclaimMode::ByCount:
      blockCount = readConfigUint(Section::FlashTranslation,
                                  Config::Key::GCReclaimBlocks);
      break;
    case Config::GCBlockReclaimMode::ByRatio:
      blockRatio = readConfigFloat(Section::FlashTranslation,
                                   Config::Key::GCReclaimThreshold);
      break;
  }

  dchoice =
      readConfigUint(Section::FlashTranslation, Config::Key::DChoiceParam);
  gcThreshold =
      readConfigFloat(Section::FlashTranslation, Config::Key::GCThreshold);
}

BasicAllocator::~BasicAllocator() {
  free(eraseCountList);
  free(inUseBlockMap);
  delete[] freeBlocks;
}

void BasicAllocator::initialize(Parameter *p) {
  AbstractAllocator::initialize(p);

  parallelism = param->parallelism / param->superpage;
  totalSuperblock = param->totalPhysicalBlocks / param->superpage;
  freeBlockCount = totalSuperblock;

  // Allocate data
  eraseCountList = (uint32_t *)calloc(totalSuperblock, sizeof(uint32_t));
  inUseBlockMap = (PPN *)calloc(parallelism, sizeof(PPN));
  freeBlocks = new std::list<PPN>[parallelism]();

  lastAllocated = 0;

  // Fill data
  uint64_t left = totalSuperblock / parallelism;

  for (uint64_t i = 0; i < parallelism; i++) {
    for (uint64_t j = 0; j < left; j++) {
      freeBlocks[i].emplace_back(i + j * parallelism);
    }
  }
}

CPU::Function BasicAllocator::allocateBlock(PPN &blockUsed) {
  CPU::Function fstat = CPU::initFunction();
  PPN idx = lastAllocated;

  if (LIKELY(blockUsed != InvalidPPN)) {
    idx = getParallelismFromSPPN(blockUsed);

    panic_if(inUseBlockMap[idx] != blockUsed, "Unexpected block ID.");

    // Insert to full block list
    uint32_t erased = eraseCountList[blockUsed];

    fullBlocks.emplace(std::make_pair(erased, blockUsed));
  }
  else {
    lastAllocated++;

    if (lastAllocated == parallelism) {
      lastAllocated = 0;
    }
  }

  panic_if(freeBlocks[idx].size() == 0, "No more free blocks at ID %" PRIu64,
           idx);

  // Allocate new block
  inUseBlockMap[idx] = std::move(freeBlocks[idx].front());
  blockUsed = inUseBlockMap[idx];

  freeBlocks[idx].pop_front();
  freeBlockCount--;

  return std::move(fstat);
}

PPN BasicAllocator::getBlockAt(PPN idx) {
  if (idx == InvalidPPN) {
    PPN ppn = inUseBlockMap[lastAllocated++];

    if (lastAllocated == parallelism) {
      lastAllocated = 0;
    }

    return ppn;
  }

  panic_if(idx >= parallelism, "Invalid parallelism index.");

  return inUseBlockMap[idx];
}

bool BasicAllocator::checkGCThreshold() {
  return (float)freeBlockCount / totalSuperblock < gcThreshold;
}

bool BasicAllocator::stopWriteRequest() {
  for (uint64_t i = 0; i < parallelism; i++) {
    if (freeBlocks[i].size() == 0) {
      return true;
    }
  }

  return false;
}

void BasicAllocator::getVictimBlocks(std::deque<PPN> &list, Event eid) {
  CPU::Function fstat = CPU::initFunction();
  uint64_t blocksToReclaim = 0;

  list.clear();

  switch (countMode) {
    case Config::GCBlockReclaimMode::ByCount:
      blocksToReclaim = blockCount;

      break;
    case Config::GCBlockReclaimMode::ByRatio:
      panic_if(totalSuperblock * blockRatio < freeBlockCount,
               "GC should not triggered.");

      blocksToReclaim =
          (uint64_t)(totalSuperblock * blockRatio) - freeBlockCount;

      break;
  }

  if (UNLIKELY(fullBlocks.size() <= blocksToReclaim * dchoice)) {
    // Copy usedBlocks to list from front (sorted!)
    for (auto &iter : fullBlocks) {
      list.emplace_back(iter.second);

      if (list.size() == blocksToReclaim) {
        break;
      }
    }
  }
  else {
    // Select victim blocks
    // We can optimize this by using lambda function, but to calculate firmware
    // latency just use switch-case statement.
    switch (selectionMode) {
      case Config::VictimSelectionMode::DChoice:
        blocksToReclaim *= dchoice;

        /* fallthrough */
      case Config::VictimSelectionMode::Random: {
        uint64_t size = fullBlocks.size();
        std::vector<uint64_t> offsets;
        std::uniform_int_distribution<uint64_t> dist(0, size - 1);

        // Collect offsets
        offsets.reserve(blocksToReclaim);

        while (offsets.size() < blocksToReclaim) {
          uint64_t idx = dist(mtengine);
          bool unique = true;

          for (auto &iter : offsets) {
            if (iter == idx) {
              unique = false;

              break;
            }
          }

          if (unique) {
            offsets.emplace_back(idx);
          }
        }

        // Select front blocksToReclaim blocks
        auto iter = fullBlocks.begin();

        for (uint64_t i = 0; i < size; i++) {
          if (i == offsets.at(list.size())) {
            list.emplace_back(iter->second);
          }

          ++iter;
        }

        if (selectionMode == Config::VictimSelectionMode::Random) {
          break;
        }
      }
      /* fallthrough */
      case Config::VictimSelectionMode::Greedy: {
        // We need valid page ratio
        std::vector<std::pair<PPN, uint32_t>> valid;

        if (list.size() == 0) {
          // Greedy
          for (auto &iter : fullBlocks) {
            valid.emplace_back(std::make_pair(
                iter.second, pMapper->getValidPages(iter.second)));
          }
        }
        else {
          // D-Choice
          valid.reserve(list.size());

          for (auto &iter : list) {
            valid.emplace_back(
                std::make_pair(iter, pMapper->getValidPages(iter)));
          }

          list.clear();
        }

        // Sort
        std::sort(valid.begin(), valid.end(),
                  [](std::pair<PPN, uint32_t> &a, std::pair<PPN, uint32_t> &b) {
                    return a.second < b.second;
                  });

        // Return front blocksToReclaim PPNs
        for (uint64_t i = 0; i < blocksToReclaim; i++) {
          list.emplace_back(valid.at(i).first);
        }
      } break;
      default:
        panic("Unexpected victim block selection mode.");
        break;
    }
  }

  scheduleFunction(CPU::CPUGroup::FlashTranslationLayer, eid, fstat);
}

void BasicAllocator::reclaimBlocks(PPN blockID, Event eid) {
  CPU::Function fstat = CPU::initFunction();

  panic_if(blockID >= totalSuperblock, "Invalid block ID.");

  // Remove PPN from full block list
  uint32_t erased = eraseCountList[blockID];
  auto range = fullBlocks.equal_range(erased);

  for (auto iter = range.first; iter != range.second; ++iter) {
    if (iter->second == blockID) {
      fullBlocks.erase(iter);
    }
  }

  // Erased!
  erased++;

  // Insert PPN to free block list
  PPN idx = getParallelismFromSPPN(blockID);
  auto fb = freeBlocks[idx].begin();

  for (; fb != freeBlocks[idx].end(); ++fb) {
    if (eraseCountList[*fb] > erased) {
      break;
    }
  }

  freeBlocks[idx].emplace(fb, blockID);

  eraseCountList[blockID] = erased;

  scheduleFunction(CPU::CPUGroup::FlashTranslationLayer, eid, fstat);
}

void BasicAllocator::getStatList(std::vector<Stat> &, std::string) noexcept {}

void BasicAllocator::getStatValues(std::vector<double> &) noexcept {}

void BasicAllocator::resetStatValues() noexcept {}

void BasicAllocator::createCheckpoint(std::ostream &out) const noexcept {
  BACKUP_SCALAR(out, parallelism);
  BACKUP_SCALAR(out, totalSuperblock);
  BACKUP_SCALAR(out, lastAllocated);
  BACKUP_BLOB(out, eraseCountList, sizeof(uint32_t) * totalSuperblock);
  BACKUP_BLOB(out, inUseBlockMap, sizeof(PPN) * parallelism);
  BACKUP_SCALAR(out, freeBlockCount);

  uint64_t size;

  for (uint64_t i = 0; i < parallelism; i++) {
    size = freeBlocks[i].size();
    BACKUP_SCALAR(out, size);

    for (auto &iter : freeBlocks[i]) {
      BACKUP_SCALAR(out, iter);
    }
  }

  size = fullBlocks.size();
  BACKUP_SCALAR(out, size);

  for (auto &iter : fullBlocks) {
    BACKUP_SCALAR(out, iter.first);
    BACKUP_SCALAR(out, iter.second);
  }

  BACKUP_SCALAR(out, selectionMode);
  BACKUP_SCALAR(out, countMode);
  BACKUP_SCALAR(out, gcThreshold);
  BACKUP_SCALAR(out, blockRatio);
  BACKUP_SCALAR(out, blockCount);
  BACKUP_SCALAR(out, dchoice);
}

void BasicAllocator::restoreCheckpoint(std::istream &in) noexcept {
  uint64_t tmp64;

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != parallelism, "FTL configuration mismatch.");

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != totalSuperblock, "FTL configuration mismatch.");

  RESTORE_SCALAR(in, lastAllocated);
  RESTORE_BLOB(in, eraseCountList, sizeof(uint32_t) * totalSuperblock);
  RESTORE_BLOB(in, inUseBlockMap, sizeof(PPN) * parallelism);
  RESTORE_SCALAR(in, freeBlockCount);

  uint64_t size;

  for (uint64_t i = 0; i < parallelism; i++) {
    RESTORE_SCALAR(in, size);

    for (uint64_t j = 0; j < size; j++) {
      PPN id;

      RESTORE_SCALAR(in, id);

      freeBlocks[i].emplace_back(id);
    }
  }

  RESTORE_SCALAR(in, size);

  for (uint64_t i = 0; i < size; i++) {
    PPN id;
    uint64_t e;

    RESTORE_SCALAR(in, e);
    RESTORE_SCALAR(in, id);

    fullBlocks.emplace(id, e);
  }

  RESTORE_SCALAR(in, selectionMode);
  RESTORE_SCALAR(in, countMode);
  RESTORE_SCALAR(in, gcThreshold);
  RESTORE_SCALAR(in, blockRatio);
  RESTORE_SCALAR(in, blockCount);
  RESTORE_SCALAR(in, dchoice);
}

}  // namespace SimpleSSD::FTL::BlockAllocator
