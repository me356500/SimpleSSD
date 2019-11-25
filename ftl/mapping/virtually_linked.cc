// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "ftl/mapping/virtually_linked.hh"

#include "ftl/allocator/abstract_allocator.hh"
#include "ftl/allocator/vl_allocator.hh"
#include "ftl/base/abstract_ftl.hh"

namespace SimpleSSD::FTL::Mapping {

VirtuallyLinked::VirtuallyLinked(ObjectData &o, CommandManager *c)
    : AbstractMapping(o, c),
      totalPhysicalSuperPages(param.totalPhysicalPages / param.superpage),
      totalPhysicalSuperBlocks(param.totalPhysicalBlocks / param.superpage),
      totalLogicalSuperPages(param.totalLogicalPages / param.superpage),
      table(nullptr),
      validEntry(totalLogicalSuperPages),
      pointer(nullptr),
      pointerValid(totalLogicalSuperPages),
      validPTE(0),
      blockMetadata(nullptr),
      clock(0) {
  // Check spare size
  panic_if(filparam->spareSize < sizeof(LPN), "NAND spare area is too small.");

  // Check superpage factor
  panic_if(param.superpage == 1,
           "Please use FTL::Mapping::PageLevel if you don't use superpage.");

  // Allocate table and block metadata
  entrySize = makeEntrySize();

  table = (uint8_t *)calloc(totalLogicalSuperPages, entrySize);
  blockMetadata = new BlockMetadata[param.totalPhysicalBlocks]();

  panic_if(!table, "Memory allocation for mapping table failed.");
  panic_if(!blockMetadata, "Memory allocation for block metadata failed.");

  tableBaseAddress =
      object.dram->allocate(totalLogicalSuperPages * entrySize,
                            "FTL::Mapping::VirtuallyLinked::Table");

  // Fill metadata
  for (uint64_t i = 0; i < param.totalPhysicalBlocks; i++) {
    blockMetadata[i] = BlockMetadata(i, filparam->page);
  }

  // Valid page bits (packed) + 2byte clock + 2byte page offset
  metadataEntrySize = DIVCEIL(filparam->page, 8) + 4;

  metadataBaseAddress =
      object.dram->allocate(totalPhysicalSuperBlocks * metadataEntrySize,
                            "FTL::Mapping::VirtuallyLinked::BlockMeta");

  // Allocate partial mapping table
  uint64_t partialTableSize = (uint64_t)(
      totalLogicalSuperPages *
      readConfigFloat(Section::FlashTranslation, Config::Key::VLTableRatio));
  mergeThreshold =
      readConfigFloat(Section::FlashTranslation, Config::Key::MergeThreshold);

  // Fill partial table
  partialTable.reserve(partialTableSize);

  for (uint64_t i = 0; i < partialTableSize; i++) {
    partialTable.emplace_back(InvalidLPN, (uint32_t)param.superpage,
                              (uint32_t)entrySize);
  }

  partialTableBaseAddress =
      object.dram->allocate(partialTableSize * entrySize * param.superpage,
                            "FTL::Mapping::VirtuallyLinked::PartialTable");

  // Allocate pointer table
  pointerSize = makePointerSize(partialTableSize);

  pointer = (uint8_t *)calloc(totalLogicalSuperPages, pointerSize);

  panic_if(!pointer, "Memory allocation for partial table pointer failed.");

  pointerBaseAddress =
      object.dram->allocate(totalLogicalSuperPages * pointerSize,
                            "FTL::Mapping::VirtuallyLinked::Pointer");
}

VirtuallyLinked::~VirtuallyLinked() {
  partialTable.clear();

  free(table);
  free(pointer);
  delete[] blockMetadata;
}

void VirtuallyLinked::physicalSuperPageStats(uint64_t &valid,
                                             uint64_t &invalid) {
  valid = 0;
  invalid = 0;

  for (uint64_t i = 0; i < param.totalPhysicalBlocks; i++) {
    auto &block = blockMetadata[i];

    if (block.nextPageToWrite > 0) {
      valid += block.validPages.count();

      for (uint32_t i = 0; i < block.nextPageToWrite; i++) {
        if (!block.validPages.test(i)) {
          ++invalid;
        }
      }
    }
  }

  // Divide by superpage
  valid /= param.superpage;
  invalid /= param.superpage;
}

// LPN -> PPN
CPU::Function VirtuallyLinked::readMappingInternal(LPN lpn, PPN &ppn) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  panic_if(lpn >= param.totalLogicalPages, "LPN out of range.");

  LPN slpn = getSLPNfromLPN(lpn);
  uint32_t sidx = (uint32_t)getSPIndexFromPPN(lpn);
  uint64_t ptr = readPointer(slpn);

  insertMemoryAddress(true, makePointerSize(slpn), pointerSize);

  if (pointerValid.test(slpn) && partialTable[ptr].isValid(sidx)) {
    PPN sppn = partialTable[ptr].getEntry(sidx);
    insertMemoryAddress(true, makePartialTableAddress(ptr, sidx), entrySize);

    ppn = sppn * param.superpage + sidx;

    PPN block = getBlockFromPPN(ppn);

    blockMetadata[block].clock = clock;
    insertMemoryAddress(false, makeBlockAddress(block), 2);
  }
  else if (validEntry.test(slpn)) {
    PPN sppn = readEntry(slpn);
    insertMemoryAddress(true, makeTableAddress(slpn), entrySize);

    ppn = sppn * param.superpage + sidx;

    PPN block = getBlockFromPPN(ppn);

    blockMetadata[block].clock = clock;
    insertMemoryAddress(false, makeBlockAddress(block), 2);
  }
  else {
    ppn = InvalidPPN;
  }

  return fstat;
}

// LPN -> PPN
CPU::Function VirtuallyLinked::writeMappingInternal(LPN lpn, bool full,
                                                    PPN &ppn, bool init) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  panic_if(lpn >= param.totalLogicalPages, "LPN out of range.");

  LPN slpn = getSLPNfromLPN(lpn);
  uint32_t sidx = (uint32_t)getSPIndexFromPPN(lpn);
  uint64_t ptr = readPointer(slpn);

  if (full) {
    if (sidx == 0) {
      // this request is full-size (superpage-size)
      if (validEntry.test(slpn)) {
        PPN sppn = readEntry(slpn);
        insertMemoryAddress(true, makeTableAddress(slpn), entrySize, !init);

        PPN pg = getPageIndexFromSPPN(sppn);

        for (uint32_t i = 0; i < param.superpage; i++) {
          PPN block = getBlockFromPPN(sppn * param.superpage + i);

          blockMetadata[block].validPages.reset(pg);
          insertMemoryAddress(false, makeBlockAddress(block) + 4 + pg / 8, 1,
                              !init);
        }
      }
      if (pointerValid.test(slpn)) {
        // Unlink
        partialTable[ptr].slpn = InvalidLPN;
        pointerValid.reset(slpn);
        validPTE--;

        // Mark as invalid
        for (uint32_t i = 0; i < param.superpage; i++) {
          if (partialTable[ptr].isValid(i)) {
            PPN sppn = partialTable[ptr].getEntry(i);
            insertMemoryAddress(true, makePartialTableAddress(ptr, i),
                                entrySize, !init);

            PPN block = getBlockFromPPN(sppn * param.superpage + i);
            PPN pg = getPageIndexFromSPPN(sppn);

            blockMetadata[block].validPages.reset(pg);
            insertMemoryAddress(false, makeBlockAddress(block) + 4 + pg / 8, 1,
                                !init);

            partialTable[ptr].resetEntry(i);
          }
        }
      }

      validEntry.set(slpn);

      // Get block from first allocated block pool
      PPN blk = allocator->getBlockAt(InvalidPPN);
      uint32_t next = blockMetadata[getBlockFromSB(blk, 0)].nextPageToWrite;

      for (uint32_t i = 1; i < param.superpage; i++) {
        panic_if(next != blockMetadata[getBlockFromSB(blk, i)].nextPageToWrite,
                 "Block metadata corrupted.");
      }

      // Check we have to get new block
      if (next == filparam->page) {
        // Get a new block
        fstat += allocator->allocateBlock(blk);

        next = 0;
      }

      ppn = makeSPPN(blk, next);

      for (uint32_t i = 0; i < param.superpage; i++) {
        PPN block = getBlockFromSB(blk, i);

        blockMetadata[block].validPages.set(next);
        insertMemoryAddress(false, makeBlockAddress(block) + 4 + next / 8, 1,
                            !init);

        blockMetadata[block].nextPageToWrite++;
        blockMetadata[block].clock = clock;
        insertMemoryAddress(false, makeBlockAddress(block), 4, !init);
      }

      // Write entry
      writeEntry(slpn, ppn);
      insertMemoryAddress(false, makeTableAddress(slpn), entrySize, !init);

      // SPPN -> PPN
      ppn *= param.superpage;
    }
    else {
      panic_if(!validEntry.test(slpn), "Not a full-size request?");

      ppn = readEntry(slpn) * param.superpage + sidx;
      // No DRAM latency: Assume all entries are written when sidx == 0
    }
  }
  else {
    if (pointerValid.test(slpn)) {
      if (partialTable[ptr].isValid(sidx)) {
        // Invalidate
        PPN sppn = partialTable[ptr].getEntry(sidx);
        insertMemoryAddress(true, makePartialTableAddress(ptr, sidx), entrySize,
                            !init);

        PPN block = getBlockFromPPN(sppn * param.superpage + sidx);
        PPN pg = getPageIndexFromSPPN(sppn);

        blockMetadata[block].validPages.reset(pg);
        insertMemoryAddress(false, makeBlockAddress(block) + 4 + pg / 8, 1,
                            !init);

        partialTable[ptr].resetEntry(sidx);
      }
    }
    else {
      // Allocate partial table
      auto iter = partialTable.begin();

      for (uint64_t i = 0; iter != partialTable.end(); ++iter, i++) {
        if (iter->slpn == InvalidLPN) {
          ptr = i;

          break;
        }
      }

      panic_if(iter == partialTable.end(), "No entry left in partial table.");

      // Link
      writePointer(slpn, ptr);
      iter->slpn = slpn;
      validPTE++;

      insertMemoryAddress(false, makePointerAddress(slpn), pointerSize, !init);

      iter->valid.reset();
    }

    // Now we have valid partial table
    pointerValid.set(slpn);

    // Get block from second allocated block pool
    PPN firstblk = ((BlockAllocator::VLAllocator *)allocator)
                       ->getPartialBlock(slpn, InvalidPPN);

    // Find writable block
    PPN blk = firstblk;

    do {
      if (blockMetadata[getBlockFromSB(blk, sidx)].nextPageToWrite !=
          filparam->page) {
        break;
      }

      blk = ((BlockAllocator::VLAllocator *)allocator)
                ->getPartialBlock(InvalidLPN, InvalidPPN);
    } while (blk != firstblk);

    if (blockMetadata[getBlockFromSB(blk, sidx)].nextPageToWrite ==
        filparam->page) {
      // Still we don't have writable block -> allocate new block
      fstat += ((BlockAllocator::VLAllocator *)allocator)
                   ->allocatePartialBlock(slpn, blk);
    }

    // Get new page
    PPN bidx = getBlockFromSB(blk, sidx);
    auto &block = blockMetadata[bidx];

    block.validPages.set(block.nextPageToWrite);
    insertMemoryAddress(false,
                        makeBlockAddress(bidx) + 4 + block.nextPageToWrite / 8,
                        1, !init);

    ppn = makeSPPN(blk, block.nextPageToWrite++);

    block.clock = clock;
    insertMemoryAddress(false, makeBlockAddress(bidx), 4, !init);

    // Write entry
    partialTable[ptr].setEntry(sidx, ppn);
    partialTable[ptr].clock = clock;

    insertMemoryAddress(false, makePartialTableAddress(ptr, sidx), entrySize,
                        !init);

    // SPPN -> PPN
    ppn = ppn * param.superpage + sidx;

    // Check partial table has all valid entry
    if (partialTable[ptr].valid.all()) {
      if (validEntry.test(slpn)) {
        PPN sppn = readEntry(slpn);
        PPN pg = getPageIndexFromSPPN(sppn);

        // Invalidate superpage
        for (uint32_t i = 0; i < param.superpage; i++) {
          blockMetadata[getBlockFromPPN(sppn * param.superpage + i)]
              .validPages.reset(pg);
        }

        validEntry.reset(slpn);
      }
    }
  }

  return fstat;
}

// LPN -> PPN
CPU::Function VirtuallyLinked::invalidateMappingInternal(LPN lpn, PPN &old) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  panic_if(lpn >= param.totalLogicalPages, "LPN out of range.");

  LPN slpn = getSLPNfromLPN(lpn);
  uint32_t sidx = (uint32_t)getSPIndexFromPPN(lpn);
  uint64_t ptr = readPointer(slpn);

  if (pointerValid.test(slpn) && partialTable[ptr].isValid(sidx)) {
    PPN sppn = partialTable[ptr].getEntry(sidx);

    partialTable[ptr].resetEntry(sidx);

    old = sppn * param.superpage + sidx;

    PPN block = getBlockFromPPN(old);
    PPN pg = getPageIndexFromSPPN(sppn);

    blockMetadata[block].validPages.reset(pg);
    insertMemoryAddress(false, makeBlockAddress(block) + 4 + pg / 8, 1);

    // Check partial table is empty
    if (partialTable[ptr].valid.none()) {
      // Unlink
      partialTable[ptr].slpn = InvalidLPN;
      pointerValid.reset(slpn);
      validPTE--;
    }
  }
  else if (validEntry.test(slpn)) {
    PPN sppn = readEntry(slpn);
    insertMemoryAddress(true, makeTableAddress(slpn), entrySize);

    if (!pointerValid.test(slpn)) {
      // Allocate partial table
      // TODO: Redundant code
      panic_if(pointerValid.all(), "No partial table entry exists.");

      auto iter = partialTable.begin();

      for (uint64_t i = 0; iter != partialTable.end(); ++iter, i++) {
        if (!pointerValid.test(i)) {
          ptr = i;

          break;
        }
      }

      // Link
      writePointer(slpn, ptr);
      iter->slpn = slpn;
      validPTE++;

      iter->valid.reset();
      pointerValid.set(slpn);
    }

    // Copy sppn to non-valid entries
    for (uint32_t i = 0; i < param.superpage; i++) {
      if (!partialTable[ptr].isValid(i)) {
        partialTable[ptr].setEntry(i, sppn);
      }
    }

    old = sppn * param.superpage + sidx;

    partialTable[ptr].resetEntry(sidx);
    blockMetadata[getBlockFromPPN(old)].validPages.reset(
        getPageIndexFromSPPN(old));

    validEntry.reset(slpn);
  }

  return fstat;
}

void VirtuallyLinked::initialize(AbstractFTL *f,
                                 BlockAllocator::AbstractAllocator *a) {
  AbstractMapping::initialize(f, a);

  panic_if(dynamic_cast<BlockAllocator::VLAllocator *>(allocator) == nullptr,
           "Requires VLAllocator as block allocator.");

  // Make first free block pool in allocator
  uint64_t parallelism = param.parallelism / param.superpage;

  for (uint64_t i = 0; i < parallelism; i++) {
    PPN tmp = InvalidPPN;

    allocator->allocateBlock(tmp);
  }

  // Make second free block pool in allocator
  for (uint64_t i = 0; i < parallelism; i++) {
    PPN tmp = InvalidPPN;

    ((BlockAllocator::VLAllocator *)allocator)
        ->allocatePartialBlock(InvalidLPN, tmp);
  }

  // Perform filling
  uint64_t nPagesToWarmup;
  uint64_t nPagesToInvalidate;
  uint64_t maxPagesBeforeGC;
  uint64_t valid;
  uint64_t invalid;
  Config::FillingType mode;
  LPN _lpn;
  PPN ppn;
  std::vector<uint8_t> spare;

  debugprint(Log::DebugID::FTL_VLFTL, "Initialization started");

  nPagesToWarmup = (uint64_t)(
      totalLogicalSuperPages *
      readConfigFloat(Section::FlashTranslation, Config::Key::FillRatio));
  nPagesToInvalidate = (uint64_t)(
      totalLogicalSuperPages * readConfigFloat(Section::FlashTranslation,
                                               Config::Key::InvalidFillRatio));
  mode = (Config::FillingType)readConfigUint(Section::FlashTranslation,
                                             Config::Key::FillingMode);
  maxPagesBeforeGC = (uint64_t)(
      filparam->page * (totalPhysicalSuperBlocks *
                        (1.f - readConfigFloat(Section::FlashTranslation,
                                               Config::Key::GCThreshold))));

  if (nPagesToWarmup + nPagesToInvalidate > maxPagesBeforeGC) {
    warn("ftl: Too high filling ratio. Adjusting invalidPageRatio.");
    nPagesToInvalidate = maxPagesBeforeGC - nPagesToWarmup;
  }

  debugprint(Log::DebugID::FTL_VLFTL, "Total logical pages: %" PRIu64,
             totalLogicalSuperPages);
  debugprint(Log::DebugID::FTL_VLFTL,
             "Total logical pages to fill: %" PRIu64 " (%.2f %%)",
             nPagesToWarmup, nPagesToWarmup * 100.f / totalLogicalSuperPages);
  debugprint(Log::DebugID::FTL_VLFTL,
             "Total invalidated pages to create: %" PRIu64 " (%.2f %%)",
             nPagesToInvalidate,
             nPagesToInvalidate * 100.f / totalLogicalSuperPages);

  // Step 1. Filling
  if (mode == Config::FillingType::SequentialSequential ||
      mode == Config::FillingType::SequentialRandom) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      for (uint32_t j = 0; j < param.superpage; j++) {
        _lpn = i * param.superpage + j;

        writeMappingInternal(_lpn, true, ppn, true);

        makeSpare(_lpn, spare);
        pFTL->writeSpare(ppn, spare);
      }
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, totalLogicalSuperPages - 1);

    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      LPN lpn = dist(gen);

      for (uint32_t j = 0; j < param.superpage; j++) {
        _lpn = lpn * param.superpage + j;

        writeMappingInternal(_lpn, true, ppn, true);

        makeSpare(_lpn, spare);
        pFTL->writeSpare(ppn, spare);
      }
    }
  }

  // Step 2. Invalidating
  if (mode == Config::FillingType::SequentialSequential) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      for (uint32_t j = 0; j < param.superpage; j++) {
        _lpn = i * param.superpage + j;

        writeMappingInternal(_lpn, true, ppn, true);

        makeSpare(_lpn, spare);
        pFTL->writeSpare(ppn, spare);
      }
    }
  }
  else if (mode == Config::FillingType::SequentialRandom) {
    // Random
    // We can successfully restrict range of LPN to create exact number of
    // invalid pages because we wrote in sequential mannor in step 1.
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nPagesToWarmup - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      LPN lpn = dist(gen);

      for (uint32_t j = 0; j < param.superpage; j++) {
        _lpn = lpn * param.superpage + j;

        writeMappingInternal(_lpn, true, ppn, true);

        makeSpare(_lpn, spare);
        pFTL->writeSpare(ppn, spare);
      }
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, totalLogicalSuperPages - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      LPN lpn = dist(gen);

      for (uint32_t j = 0; j < param.superpage; j++) {
        _lpn = lpn * param.superpage + j;

        writeMappingInternal(_lpn, true, ppn, true);

        makeSpare(_lpn, spare);
        pFTL->writeSpare(ppn, spare);
      }
    }
  }

  // Report
  physicalSuperPageStats(valid, invalid);
  debugprint(Log::DebugID::FTL_VLFTL, "Filling finished. Page status:");
  debugprint(Log::DebugID::FTL_VLFTL,
             "  Total valid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             valid, valid * 100.f / totalLogicalSuperPages, nPagesToWarmup,
             (int64_t)(valid - nPagesToWarmup));
  debugprint(Log::DebugID::FTL_VLFTL,
             "  Total invalid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             invalid, invalid * 100.f / totalLogicalSuperPages,
             nPagesToInvalidate, (int64_t)(invalid - nPagesToInvalidate));
  debugprint(Log::DebugID::FTL_VLFTL, "Initialization finished");
}

// LPN + NLP
LPN VirtuallyLinked::getPageUsage(LPN slpn, LPN nlp) {
  LPN count = 0;

  // Convert to SLPN
  slpn /= param.superpage;
  nlp = DIVCEIL(nlp, param.superpage);

  panic_if(slpn + nlp > totalLogicalSuperPages, "LPN out of range.");

  for (LPN i = slpn; i < nlp; i++) {
    if (validEntry.test(i)) {
      count++;
    }
  }

  // Convert to LPN
  return count * param.superpage;
}

// PPN
uint32_t VirtuallyLinked::getValidPages(PPN ppn) {
  return (uint32_t)blockMetadata[getBlockFromPPN(ppn)].validPages.count();
}

// PPN
uint16_t VirtuallyLinked::getAge(PPN ppn) {
  return (uint16_t)(clock - blockMetadata[getBlockFromPPN(ppn)].clock);
}

CPU::Function VirtuallyLinked::readMapping(Command &cmd) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  clock++;

  panic_if(cmd.subCommandList.size() != cmd.length, "Unexpected sub commands.");

  // Perform read translation
  for (auto &scmd : cmd.subCommandList) {
    if (UNLIKELY(scmd.lpn == InvalidLPN)) {
      continue;
    }

    fstat += readMappingInternal(scmd.lpn, scmd.ppn);

    debugprint(Log::DebugID::FTL_VLFTL,
               "Read  | LPN %" PRIx64 "h -> PPN %" PRIx64 "h", scmd.lpn,
               scmd.ppn);
  }

  return fstat;
}

CPU::Function VirtuallyLinked::writeMapping(Command &cmd) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  clock++;

  panic_if(cmd.subCommandList.size() != cmd.length, "Unexpected sub commands.");

  // Check command
  if (UNLIKELY(cmd.offset == InvalidLPN)) {
    // This is GC write request and this request must have spare field
    auto iter = cmd.subCommandList.begin();

    iter->lpn = readSpare(iter->spare);
    LPN slpn = getSLPNfromLPN(iter->lpn);

    cmd.offset = iter->lpn;

    // Read all
    ++iter;

    for (; iter != cmd.subCommandList.end(); ++iter) {
      iter->lpn = readSpare(iter->spare);

      panic_if(slpn != getSLPNfromLPN(iter->lpn),
               "Command has two or more superpages.");
    }
  }

  // Check align
  LPN alignedBegin = cmd.offset / param.superpage * param.superpage;
  LPN alignedEnd =
      DIVCEIL(cmd.offset + cmd.length, param.superpage) * param.superpage;
  bool aligned =
      alignedBegin == cmd.offset && cmd.offset + cmd.length == alignedEnd;

  // Perform write translation
  for (auto &scmd : cmd.subCommandList) {
    fstat += writeMappingInternal(scmd.lpn, aligned, scmd.ppn);

    makeSpare(scmd.lpn, scmd.spare);

    debugprint(Log::DebugID::FTL_VLFTL,
               "Write | LPN %" PRIx64 "h -> PPN %" PRIx64 "h", scmd.lpn,
               scmd.ppn);
  }

  return fstat;
}

CPU::Function VirtuallyLinked::invalidateMapping(Command &cmd) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  clock++;

  panic_if(cmd.subCommandList.size() > 0, "Unexpected sub commands.");

  cmd.subCommandList.reserve(cmd.length);

  // Perform invalidation
  LPN lpn = cmd.offset;
  PPN ppn = InvalidPPN;

  do {
    fstat += invalidateMappingInternal(lpn, ppn);

    debugprint(Log::DebugID::FTL_VLFTL,
               "Trim/Format | LPN %" PRIx64 "h -> PPN %" PRIx64 "h", lpn, ppn);

    // Add subcommand
    commandManager->appendTranslation(cmd, lpn, ppn);

    lpn++;
  } while (lpn < cmd.offset + cmd.length);

  return fstat;
}

// SPPN
void VirtuallyLinked::getCopyList(CopyList &copy, Event eid) {
  CPU::Function fstat;
  CPU::markFunction(fstat);

  panic_if(copy.blockID >= totalPhysicalSuperBlocks, "Block out of range.");

  // For the valid pages in target block, create I/O operation
  copy.commandList.reserve(filparam->page);

  for (uint32_t i = 0; i < filparam->page; i++) {
    uint64_t tag = pFTL->makeFTLCommandTag();
    auto &copycmd = commandManager->createFTLCommand(tag);

    copycmd.offset = InvalidLPN;  // writeMapping will fill this
    copycmd.length = 0;

    for (uint64_t j = 0; j < param.superpage; j++) {
      auto block = &blockMetadata[copy.blockID * param.superpage + j];

      if (block->validPages.test(i)) {
        copycmd.length++;

        // At this stage, we don't know LPN
        commandManager->appendTranslation(copycmd, InvalidLPN,
                                          makePPN(copy.blockID, j, i));
      }
      else {
        commandManager->appendTranslation(copycmd, InvalidLPN, InvalidPPN);
      }
    }

    if (copycmd.length > 0) {
      copy.commandList.emplace_back(tag);
    }
    else {
      commandManager->destroyCommand(tag);
    }
  }

  // For the target block, create erase operation
  copy.eraseTag = pFTL->makeFTLCommandTag();

  auto &erasecmd = commandManager->createFTLCommand(copy.eraseTag);

  erasecmd.offset = InvalidLPN;  // Erase has no LPN
  erasecmd.length = param.superpage;

  for (uint32_t i = 0; i < param.superpage; i++) {
    commandManager->appendTranslation(erasecmd, InvalidLPN,
                                      makePPN(copy.blockID, i, 0));
  }

  scheduleFunction(CPU::CPUGroup::FlashTranslationLayer, eid, fstat);
}

// SPPN
void VirtuallyLinked::releaseCopyList(CopyList &copy) {
  // Destroy all commands
  for (auto &iter : copy.commandList) {
    commandManager->destroyCommand(iter);
  }

  commandManager->destroyCommand(copy.eraseTag);

  // Mark block as erased
  for (uint32_t i = 0; i < param.superpage; i++) {
    blockMetadata[copy.blockID * param.superpage + i].nextPageToWrite = 0;
    blockMetadata[copy.blockID * param.superpage + i].validPages.reset();
  }

  debugprint(Log::DebugID::FTL_VLFTL, "Erase | SPPN %" PRIx64 "h",
             copy.blockID);
}

bool VirtuallyLinked::triggerMerge() {
  return (float)validPTE / partialTable.size() >= mergeThreshold;
}

uint64_t VirtuallyLinked::getMergeReadCommand() {
  uint64_t tag = pFTL->makeFTLCommandTag();

  panic_if(validPTE == 0, "No partial table entry exists.");

  // Clock PLRU
  uint16_t diff = 0;
  uint64_t size = partialTable.size();
  uint64_t idx = size;

  for (uint64_t i = 0; i < size; i++) {
    auto &iter = partialTable[i];

    if (iter.slpn != InvalidLPN && diff < (uint64_t)(clock - iter.clock)) {
      diff = clock - iter.clock;
      idx = i;
    }
  }

  // Check switch merge
  auto &iter = partialTable[idx];

  if (iter.valid.all()) {
    bool ok = true;
    LPN sppn = iter.getEntry(0);

    for (uint32_t i = 1; i < param.superpage; i++) {
      if (sppn != iter.getEntry(i)) {
        ok = false;

        break;
      }
    }

    if (ok) {
      debugprint(Log::DebugID::FTL_VLFTL,
                 "Merge | PPN %" PRIx64 "h (SLPN %" PRIx64 "h) switch merged",
                 sppn, iter.slpn);

      // Reconstruct table
      validEntry.set(iter.slpn);
      writeEntry(iter.slpn, sppn);

      // Invalidate partial table entry
      pointerValid.reset(iter.slpn);
      iter.slpn = InvalidLPN;

      return 0;
    }
  }

  debugprint(Log::DebugID::FTL_VLFTL, "Merge | SLPN %" PRIx64 "h", iter.slpn);

  PPN sppn = InvalidPPN;
  PPN ppn = InvalidPPN;

  panic_if(!pointerValid.test(iter.slpn) || readPointer(iter.slpn) != idx,
           "Partial table corrupted.");

  if (validEntry.test(iter.slpn)) {
    sppn = readEntry(iter.slpn);
  }

  // Create Command
  auto &cmd = commandManager->createFTLCommand(tag);

  cmd.offset = iter.slpn;
  cmd.length = 0;

  for (uint32_t i = 0; i < param.superpage; i++) {
    if (iter.isValid(i)) {
      cmd.length++;
      ppn = iter.getEntry(i) * param.superpage + i;

      commandManager->appendTranslation(cmd, InvalidLPN, ppn);

      debugprint(Log::DebugID::FTL_VLFTL,
                 "Merge | Read  | %u: PPN %" PRIx64 "h from partial table", i,
                 ppn);
    }
    else if (sppn != InvalidPPN) {
      cmd.length++;
      ppn = sppn * param.superpage + i;

      commandManager->appendTranslation(cmd, InvalidLPN, ppn);

      debugprint(Log::DebugID::FTL_VLFTL,
                 "Merge | Read  | %u: PPN %" PRIx64 "h from mapping table", i,
                 ppn);
    }
    else {
      commandManager->appendTranslation(cmd, InvalidLPN, InvalidPPN);

      debugprint(Log::DebugID::FTL_VLFTL, "Merge | Read  | %u not valid", i);
    }
  }

  return tag;
}

uint64_t VirtuallyLinked::getMergeWriteCommand(uint64_t tag) {
  auto &cmd = commandManager->getCommand(tag);

  // Validate and fill LPN
  uint32_t found = 0;

  for (LPN i = 0; i < param.superpage; i++) {
    auto &scmd = cmd.subCommandList.at(i);

    if (scmd.ppn != InvalidPPN) {
      found++;

      scmd.lpn = readSpare(scmd.spare);

      panic_if(cmd.offset != getSLPNfromLPN(scmd.lpn),
               "Partial table corrupted.");
    }
  }

  panic_if(found != cmd.length || found == 0, "Command not completed.");

  // Write mapping
  for (LPN i = 0; i < param.superpage; i++) {
    auto &scmd = cmd.subCommandList.at(i);
    PPN old = scmd.ppn;

    if (old == InvalidPPN) {
      scmd.lpn = cmd.offset * param.superpage + i;
    }

    writeMappingInternal(scmd.lpn, true, scmd.ppn);

    if (LIKELY(old != InvalidPPN)) {
      debugprint(Log::DebugID::FTL_VLFTL,
                 "Merge | Write | %u: PPN %" PRIx64 "h (LPN %" PRIx64
                 "h) -> PPN %" PRIx64 "h",
                 i, old, scmd.lpn, scmd.ppn);
    }
    else {
      debugprint(Log::DebugID::FTL_VLFTL,
                 "Merge | Write | %u: Invalid (LPN %" PRIx64
                 "h) -> PPN %" PRIx64 "h",
                 i, scmd.lpn, scmd.ppn);
    }
  }

  return tag;
}

void VirtuallyLinked::destroyMergeCommand(uint64_t tag) {
  commandManager->destroyCommand(tag);
}

void VirtuallyLinked::getStatList(std::vector<Stat> &, std::string) noexcept {}

void VirtuallyLinked::getStatValues(std::vector<double> &) noexcept {}

void VirtuallyLinked::resetStatValues() noexcept {}

void VirtuallyLinked::createCheckpoint(std::ostream &out) const noexcept {
  BACKUP_SCALAR(out, totalPhysicalSuperPages);
  BACKUP_SCALAR(out, totalPhysicalSuperBlocks);
  BACKUP_SCALAR(out, totalLogicalSuperPages);
  BACKUP_SCALAR(out, entrySize);
  BACKUP_SCALAR(out, pointerSize);
  BACKUP_BLOB(out, table, totalLogicalSuperPages * entrySize);
  BACKUP_BLOB(out, pointer, totalLogicalSuperPages * pointerSize);
  BACKUP_SCALAR(out, validPTE);
  BACKUP_SCALAR(out, clock);

  validEntry.createCheckpoint(out);
  pointerValid.createCheckpoint(out);

  for (auto &iter : partialTable) {
    BACKUP_SCALAR(out, iter.slpn);
    BACKUP_BLOB(out, iter.data, iter.superpage * iter.entrySize);

    iter.valid.createCheckpoint(out);
  }

  for (uint64_t i = 0; i < param.totalPhysicalBlocks; i++) {
    BACKUP_SCALAR(out, blockMetadata[i].nextPageToWrite);
    BACKUP_SCALAR(out, blockMetadata[i].clock);

    blockMetadata[i].validPages.createCheckpoint(out);
  }
}

void VirtuallyLinked::restoreCheckpoint(std::istream &in) noexcept {
  uint64_t tmp64;

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != totalPhysicalSuperPages,
           "Invalid FTL configuration while restore.");

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != totalPhysicalSuperBlocks,
           "Invalid FTL configuration while restore.");

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != totalLogicalSuperPages,
           "Invalid FTL configuration while restore.");

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != entrySize, "Invalid FTL configuration while restore.");

  RESTORE_SCALAR(in, tmp64);
  panic_if(tmp64 != pointerSize, "Invalid FTL configuration while restore.");

  RESTORE_BLOB(in, table, totalLogicalSuperPages * entrySize);
  RESTORE_BLOB(in, pointer, totalLogicalSuperPages * pointerSize);
  RESTORE_SCALAR(in, validPTE);
  RESTORE_SCALAR(in, clock);

  validEntry.restoreCheckpoint(in);
  pointerValid.restoreCheckpoint(in);

  for (auto &iter : partialTable) {
    RESTORE_SCALAR(in, iter.slpn);
    RESTORE_BLOB(in, iter.data, iter.superpage * iter.entrySize);

    iter.valid.restoreCheckpoint(in);
  }

  for (uint64_t i = 0; i < totalPhysicalSuperBlocks; i++) {
    RESTORE_SCALAR(in, blockMetadata[i].nextPageToWrite);
    RESTORE_SCALAR(in, blockMetadata[i].clock);

    blockMetadata[i].validPages.restoreCheckpoint(in);
  }
}

}  // namespace SimpleSSD::FTL::Mapping
