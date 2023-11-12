/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ftl/page_mapping.hh"

#include <algorithm>
#include <limits>
#include <random>

#include "util/algorithm.hh"
#include "util/bitset.hh"

namespace SimpleSSD {

namespace FTL {

PageMapping::PageMapping(ConfigReader &c, Parameter &p, PAL::PAL *l,
                         DRAM::AbstractDRAM *d)
    : AbstractFTL(p, l, d),
      pPAL(l),
      conf(c),
      lastFreeBlock(param.pageCountToMaxPerf),
      lastFreeBlockIOMap(param.ioUnitInPage),
      bReclaimMore(false) {
  blocks.reserve(param.totalPhysicalBlocks);
  table.reserve(param.totalLogicalBlocks * param.pagesInBlock * 32);

  for (uint32_t i = 0; i < param.totalPhysicalBlocks; i++) {
    freeBlocks.emplace_back(Block(i, param.pagesInBlock, param.ioUnitInPage));
  }
  nFreeBlocks = param.totalPhysicalBlocks;

  status.totalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;

  // Allocate free blocks
  //for (uint32_t i = 0; i < param.pageCountToMaxPerf; i++) {
  //  lastFreeBlock.at(i) = getFreeBlock(i);
  //}

  lastFreeBlockIndex = 0;

  memset(&stat, 0, sizeof(stat));
  stat.MGCcount = 0;
  stat.MGCcopydata = 0;
  bRandomTweak = conf.readBoolean(CONFIG_FTL, FTL_USE_RANDOM_IO_TWEAK);
  bitsetSize = bRandomTweak ? param.ioUnitInPage : 1;
}

PageMapping::~PageMapping() {}

void PageMapping::getBlockStatus() {

  auto block = blocks.find(spareblk_idx);

  cout << "Free : " << nFreeBlocks << "\n";
  cout << "Block idx : " << spareblk_idx << "\n";
  cout << "Parityidx : " << spareblk_idx % 32 << "\n";
  cout << "blkparity : " << block->second.getParityChannelIndex() << "\n";
  cout << "SBtype : " << block->second.getSBtype() << "\n";
  cout << "Writing pages : " << block->second.getValidPageCountRaw() << "\n";
  cout << "IS full : " << block->second.isfull() << "\n";

  for (int i = 0; i < 32; ++i) {

    cout << "channel " << i << " : " << block->second.getNextWritePageIndex(i) << "\n";
  
  }

#ifdef DEBUG
  fgetc(stdin);
#endif
  
}

bool PageMapping::initialize() {
  uint64_t nPagesToWarmup;
  uint64_t nPagesToInvalidate;
  uint64_t nTotalLogicalPages;
  uint64_t maxPagesBeforeGC;
  uint64_t tick;
  uint64_t valid;
  uint64_t invalid;
  FILLING_MODE mode;

  Request req(param.ioUnitInPage);

  debugprint(LOG_FTL_PAGE_MAPPING, "Initialization started");

  nTotalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;
  nPagesToWarmup =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_FILL_RATIO);
  nPagesToInvalidate =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_INVALID_PAGE_RATIO);
  mode = (FILLING_MODE)conf.readUint(CONFIG_FTL, FTL_FILLING_MODE);
  maxPagesBeforeGC =
      param.pagesInBlock *
      (param.totalPhysicalBlocks *
           (1 - conf.readFloat(CONFIG_FTL, FTL_GC_THRESHOLD_RATIO)) -
       param.pageCountToMaxPerf);  // # free blocks to maintain

  if (nPagesToWarmup + nPagesToInvalidate > maxPagesBeforeGC) {
    warn("ftl: Too high filling ratio. Adjusting invalidPageRatio.");
    nPagesToInvalidate = maxPagesBeforeGC - nPagesToWarmup;
  }

  debugprint(LOG_FTL_PAGE_MAPPING, "Total logical pages: %" PRIu64,
             nTotalLogicalPages);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "Total logical pages to fill: %" PRIu64 " (%.2f %%)",
             nPagesToWarmup, nPagesToWarmup * 100.f / nTotalLogicalPages);
  debugprint(LOG_FTL_PAGE_MAPPING,
             "Total invalidated pages to create: %" PRIu64 " (%.2f %%)",
             nPagesToInvalidate,
             nPagesToInvalidate * 100.f / nTotalLogicalPages);

  req.ioFlag.set();

  lpn_channel.clear();
  warmup = 1;
  lpn_channel = vector<uint32_t>(68000000, 32);
  GCbuf.clear();
  GCbuf_seg.clear();
#ifndef skip_invalidGCbuffer
  GCbuf.reserve(GCbufSize);
  GCbuf_seg.reserve(GCbufSize);
#endif
  writeBuf.clear();
  writeBuf.reserve(writeBufSize);
  writeBufVertical.clear();
  writeBufVertical.reserve(writeBufSize);
  segSB_time = 0;
  segSB_weight.clear();
  parity_cnt = 0;
  cur_tick = 0;
  parity_write = 0;
  GCcopypage = 0;
  pwrite = vector<uint32_t>(32, 0);
  spareblk_idx = 9999999;
  MGCcount = 0;
  // Step 1. Filling
  if (mode == FILLING_MODE_0 || mode == FILLING_MODE_1) {
    // Sequential
    for (uint64_t i = 1; i <= 61757952; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal(req, tick, false, 0);
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick, false);
    }
  }

  // Step 2. Invalidating
  if (mode == FILLING_MODE_0) {
    // Sequential
    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal(req, tick, false);
    }
  }
  else if (mode == FILLING_MODE_1) {
    // Random
    // We can successfully restrict range of LPN to create exact number of
    // invalid pages because we wrote in sequential mannor in step 1.
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nPagesToWarmup - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick, false);
    }
  }
  else {
    // Random
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick, false);
    }
  }

  warmup = 0;

  // Report
  calculateTotalPages(valid, invalid);
  debugprint(LOG_FTL_PAGE_MAPPING, "Filling finished. Page status:");
  debugprint(LOG_FTL_PAGE_MAPPING,
             "  Total valid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             valid, valid * 100.f / nTotalLogicalPages, nPagesToWarmup,
             (int64_t)(valid - nPagesToWarmup));
  debugprint(LOG_FTL_PAGE_MAPPING,
             "  Total invalid physical pages: %" PRIu64
             " (%.2f %%, target: %" PRIu64 ", error: %" PRId64 ")",
             invalid, invalid * 100.f / nTotalLogicalPages, nPagesToInvalidate,
             (int64_t)(invalid - nPagesToInvalidate));
  debugprint(LOG_FTL_PAGE_MAPPING, "Initialization finished");

  return true;
}

void PageMapping::read(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (lpn_channel[req.lpn] != IN_GCBUFFER && lpn_channel[req.lpn] != IN_TPBUFFER) {

    req.ioFlag.reset();
    req.ioFlag.set(lpn_channel[req.lpn]);

    if (req.ioFlag.count() > 0) {
      readInternal(req, tick);

      debugprint(LOG_FTL_PAGE_MAPPING,
                  "READ  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
                  ")",
                  req.lpn, begin, tick, tick - begin);
    }
    else {
      warn("FTL got empty request");
    }

  }
  

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ);
}


void PageMapping::write(Request &req, uint64_t &tick, bool SBtype) {
  uint64_t begin = tick;

#ifndef vertical
  SBtype = 0;
#endif

#ifdef writebuffer
  if (warmup) {
    if (req.ioFlag.count() > 0) {
      writeInternal(req, tick);

      debugprint(LOG_FTL_PAGE_MAPPING,
                "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
                ")",
                req.lpn, begin, tick, tick - begin);
    }
    else {
      warn("FTL got empty request");
    }
  }
  else {

    if (SBtype == 0)
      writeBuf.emplace_back(req);
    else if (SBtype == 1)
      writeBufVertical.emplace_back(req);

    auto &curBuf = (SBtype ? writeBufVertical : writeBuf);

    if (curBuf.size() == writeBufSize) {

      for (auto &i : curBuf) {

        if (i.ioFlag.count() > 0) {
          writeInternal(i, tick, 1, SBtype);

          debugprint(LOG_FTL_PAGE_MAPPING,
                    "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
                    ")",
                    i.lpn, begin, tick, tick - begin);
        }
        else {
          warn("FTL got empty request");
        }
      }
      
#ifdef DEBUG
      auto block = blocks.find(spareblk_idx);
      if (!block->second.isfull() && !block->second.isempty()) {
        cout << "After flush horizontal\n";
        getBlockStatus();
      }
#endif 
      if (SBtype == 0) {
        writeBuf.clear();
        writeBuf.reserve(writeBufSize);
      }
      else if (SBtype == 1) {
        writeBufVertical.clear();
        writeBufVertical.reserve(writeBufSize);
      }
      
    }
  }
  
#else
  if (req.ioFlag.count() > 0) {
    writeInternal(req, tick);

    debugprint(LOG_FTL_PAGE_MAPPING,
               "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }
#endif
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE);
}


void PageMapping::trim(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  trimInternal(req, tick);

  debugprint(LOG_FTL_PAGE_MAPPING,
             "TRIM  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
             ")",
             req.lpn, begin, tick, tick - begin);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM);
}

void PageMapping::format(LPNRange &range, uint64_t &tick) {
  PAL::Request req(param.ioUnitInPage);
  std::vector<uint32_t> list;

  req.ioFlag.set();

  for (auto iter = table.begin(); iter != table.end();) {
    if (iter->first >= range.slpn && iter->first < range.slpn + range.nlp) {
      auto &mappingList = iter->second;

      // Do trim
      /*
      for (uint32_t idx = 0; idx < bitsetSize; idx++) {
        auto &mapping = mappingList.at(idx);
        auto block = blocks.find(mapping.first);

        if (block == blocks.end()) {
          panic("Block is not in use");
        }

        block->second.invalidate(mapping.second, idx);

        // Collect block indices
        list.push_back(mapping.first);
      }*/
      auto &mapping = mappingList;
      auto block = blocks.find(mapping.first);

      if (block == blocks.end()) {
        panic("Block is not in use");
      }

      block->second.invalidate(mapping.second, lpn_channel[iter->first]);

      // Collect block indices
      list.push_back(mapping.first);

      iter = table.erase(iter);
    }
    else {
      iter++;
    }
  }

  // Get blocks to erase
  std::sort(list.begin(), list.end());
  auto last = std::unique(list.begin(), list.end());
  list.erase(last, list.end());
  // Do GC only in specified blocks
  uint32_t MGC = 0;
  doGarbageCollection(list, tick, MGC);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::FORMAT);
}

Status *PageMapping::getStatus(uint64_t lpnBegin, uint64_t lpnEnd) {
  status.freePhysicalBlocks = nFreeBlocks;

  if (lpnBegin == 0 && lpnEnd >= status.totalLogicalPages) {
    status.mappedLogicalPages = table.size();
  }
  else {
    status.mappedLogicalPages = 0;

    for (uint64_t lpn = lpnBegin; lpn < lpnEnd; lpn++) {
      if (table.count(lpn) > 0) {
        status.mappedLogicalPages++;
      }
    }
  }

  return &status;
}

float PageMapping::freeBlockRatio() {
  return (float)nFreeBlocks / param.totalPhysicalBlocks;
}

uint32_t PageMapping::convertBlockIdx(uint32_t blockIdx) {
  return blockIdx % param.pageCountToMaxPerf;
}

uint32_t PageMapping::getFreeBlock(uint32_t idx) {
  uint32_t blockIndex = 0;

  if (idx >= param.pageCountToMaxPerf) {
    panic("Index out of range");
  }

  if (nFreeBlocks > 0) {
    // Search block which is blockIdx % param.pageCountToMaxPerf == idx
    auto iter = freeBlocks.begin();

    for (; iter != freeBlocks.end(); iter++) {
      blockIndex = iter->getBlockIndex();

      break;
      //if (blockIndex % param.pageCountToMaxPerf == idx) {
      //  break;
      //}
    }

    // Sanity check
    if (iter == freeBlocks.end()) {
      // Just use first one
      iter = freeBlocks.begin();
      blockIndex = iter->getBlockIndex();
    }

    // Insert found block to block list
    if (blocks.find(blockIndex) != blocks.end()) {
      panic("Corrupted");
    }

    blocks.emplace(blockIndex, std::move(*iter));

    // Remove found block from free block list
    freeBlocks.erase(iter);
    nFreeBlocks--;
  }
  else {
    panic("No free block left");
  }

  return blockIndex;
}

uint32_t PageMapping::getLastFreeBlock(Bitset &iomap) {
  if (!bRandomTweak || (lastFreeBlockIOMap & iomap).any()) {
    // Update lastFreeBlockIndex
    lastFreeBlockIndex++;

    if (lastFreeBlockIndex == param.pageCountToMaxPerf) {
      lastFreeBlockIndex = 0;
    }

    lastFreeBlockIOMap = iomap;
  }
  else {
    lastFreeBlockIOMap |= iomap;
  }

  auto freeBlock = blocks.find(lastFreeBlock.at(lastFreeBlockIndex));

  // Sanity check
  if (freeBlock == blocks.end()) {
    panic("Corrupted");
  }

#ifdef parity_rotate
  // If current free block parity index matches iomap
  uint32_t parity_idx = freeBlock->second.getparityChannelIndex();
  Bitset iomask(32);
  iomask.reset();
  iomask.set(parity_idx);

  // parity channel conflict
  while (parity_cnt && (iomask & iomap).any()) {

    lastFreeBlockIOMap = iomap;

    lastFreeBlockIndex++;

    if (lastFreeBlockIndex == param.pageCountToMaxPerf) {
      lastFreeBlockIndex = 0;
    }

    freeBlock = blocks.find(lastFreeBlock.at(lastFreeBlockIndex));

    parity_idx = freeBlock->second.getparityChannelIndex();

    iomask.reset();
    iomask.set(parity_idx);

  }
#endif

  // If current free block is full, get next block
  if (freeBlock->second.getNextWritePageIndex() == param.pagesInBlock) {
    lastFreeBlock.at(lastFreeBlockIndex) = getFreeBlock(lastFreeBlockIndex);

    bReclaimMore = true;
  }

  return lastFreeBlock.at(lastFreeBlockIndex);
}

// calculate weight of each block regarding victim selection policy
void PageMapping::calculateVictimWeight(
    std::vector<std::pair<uint32_t, float>> &weight, const EVICT_POLICY policy,
    uint64_t tick) {
  float temp;

  weight.reserve(blocks.size());
  
#ifdef segSB

  segSB_weight.clear();
  segSB_weight.resize(MGC_segments);


#endif


  switch (policy) {
    case POLICY_GREEDY:
    case POLICY_RANDOM:
    case POLICY_DCHOICE:
      for (auto &iter : blocks) {

        if (!iter.second.isfull()) {
          continue;
        }

#ifdef segSB
        if (iter.second.getSBtype() == 1) {

          for (int i = 0; i < MGC_segments; ++i) {

            segSB_weight[i].emplace_back(iter.first, iter.second.getPartialValidPageCount(i));
          }

        }
        else if (iter.second.getSBtype() == 0) {
          weight.emplace_back(iter.first, iter.second.getValidPageCountRaw());
        }

#endif
        
      }

      break;
    case POLICY_COST_BENEFIT:
      for (auto &iter : blocks) {
        if (iter.second.getNextWritePageIndex() != param.pagesInBlock) {
          continue;
        }

        temp = (float)(iter.second.getValidPageCountRaw()) / param.pagesInBlock;

        weight.push_back(
            {iter.first,
             temp / ((1 - temp) * (tick - iter.second.getLastAccessedTime()))});
      }

      break;
    default:
      panic("Invalid evict policy");
  }
}

void PageMapping::exchange_seg(uint32_t &dist, uint32_t &src, uint32_t seg_id) {
  if (src == dist)
    return;

  auto dist_block = blocks.find(dist);
  auto src_block = blocks.find(src);
  uint32_t channel = seg_id * (blk_per_superblk / MGC_segments);
  // pageindex, channel idx

  for (uint32_t i = 0; i < 8; ++i)  {

    uint32_t idx = channel + i;

    for (uint32_t pageIndex = 0; pageIndex < param.pagesInBlock; ++pageIndex) {
     
      uint64_t dist_lpn = dist_block->second.getppLPN(pageIndex, idx);
      uint64_t src_lpn = src_block->second.getppLPN(pageIndex, idx);
      dist_block->second.setppLPN(pageIndex, idx, src_lpn);
      src_block->second.setppLPN(pageIndex, idx, dist_lpn);

      bool dist_valid = dist_block->second.getValidBits(pageIndex, idx);
      bool src_valid = src_block->second.getValidBits(pageIndex, idx);
      dist_block->second.setValidBits(pageIndex, idx, src_valid);
      src_block->second.setValidBits(pageIndex, idx, dist_valid);

      // need to update mapping
      auto dist_map = table.find(dist_lpn);

      if (dist_map != table.end() && dist_valid) {
        table.at(dist_lpn) = make_pair(src_block->first, pageIndex);
      }
    }
    
  }
}

void PageMapping::selectVictimBlock(std::vector<uint32_t> &list,
                                    uint64_t &tick, uint32_t &MGC) {
  static const GC_MODE mode = (GC_MODE)conf.readInt(CONFIG_FTL, FTL_GC_MODE);
  static const EVICT_POLICY policy =
      (EVICT_POLICY)conf.readInt(CONFIG_FTL, FTL_GC_EVICT_POLICY);
  static uint32_t dChoiceParam =
      conf.readUint(CONFIG_FTL, FTL_GC_D_CHOICE_PARAM);
  uint64_t nBlocks = conf.readUint(CONFIG_FTL, FTL_GC_RECLAIM_BLOCK);
  std::vector<std::pair<uint32_t, float>> weight;

  list.clear();

  // Calculate number of blocks to reclaim
  if (mode == GC_MODE_0) {
    // DO NOTHING
  }
  else if (mode == GC_MODE_1) {
    static const float t = conf.readFloat(CONFIG_FTL, FTL_GC_RECLAIM_THRESHOLD);

    nBlocks = param.totalPhysicalBlocks * t - nFreeBlocks;
  }
  else {
    panic("Invalid GC mode");
  }

  // reclaim one more if last free block fully used
  if (bReclaimMore) {
    nBlocks += param.pageCountToMaxPerf;

    bReclaimMore = false;
  }

  //struct timeval start, end, elapsed;
  //gettimeofday(&start, NULL);

  // Calculate weights of all blocks
  calculateVictimWeight(weight, policy, tick);

  if (policy == POLICY_RANDOM || policy == POLICY_DCHOICE) {
    uint64_t randomRange =
        policy == POLICY_RANDOM ? nBlocks : dChoiceParam * nBlocks;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dist(0, weight.size() - 1);
    std::vector<std::pair<uint32_t, float>> selected;

    while (selected.size() < randomRange) {
      uint64_t idx = dist(gen);

      if (weight.at(idx).first < std::numeric_limits<uint32_t>::max()) {
        selected.push_back(weight.at(idx));
        weight.at(idx).first = std::numeric_limits<uint32_t>::max();
      }
    }

    weight = std::move(selected);
  }

  // Sort weights
  std::sort(
      weight.begin(), weight.end(),
      [](const std::pair<uint32_t, float> &a, const std::pair<uint32_t, float> &b) -> bool {
        return a.second < b.second;
      });

  // Select victims from the blocks with the lowest weight
  nBlocks = MIN(nBlocks, weight.size());
  
  /* 
    weight : block index , page valid count
    parity SB mean parity Channel
  
  */
#ifdef segSB
  

  for (int i = 0; i < MGC_segments; ++i)
  {
    sort(segSB_weight[i].begin(), segSB_weight[i].end(),
    [](const pair<uint32_t, uint32_t> &l, const pair<uint32_t, uint32_t> &r) ->bool {
      return l.second < r.second;
    });
  }


  vector<uint32_t> paritySBCount(blk_per_superblk, 0);
  vector<pair<uint32_t, uint32_t>> MGCcandidate(MGC_segments);
  uint32_t parityBlkIdx = 0;
  uint32_t maxCount = 0;
  uint32_t MGCcopydata = 0;
  // MLCgetGCvictimGreedy.c:50
  // Greedy select 4 segments
  for (uint32_t i = 0; i < MGC_segments; ++i) 
  {
    MGCcandidate[i] = (segSB_weight[i][0]);
    MGCcopydata += MGCcandidate[i].second;
  }

  // MLCgetGCvictimGreedy.c:148
  for (uint32_t i = 0; i < MGC_segments; ++i) 
  {
    paritySBCount[MGCcandidate[i].first % blk_per_superblk]++;
  }

  // MLCgetGCvictimGreedy.c:149
  // find the most concentrated parity channel
  for (uint32_t i = 0; i < blk_per_superblk; ++i) 
  {
    if (paritySBCount[i] > maxCount) 
    {
      maxCount = paritySBCount[i];
      parityBlkIdx = i;
    }
  }

  // MLCgetGCvictimGreedy.c:157 same_chip_parity
  // make sure all candidate have same parity block index (channel)
  for (uint32_t i = 0; i < MGC_segments; ++i) 
  {

    if (MGCcandidate[i].first % blk_per_superblk != parityBlkIdx) 
    {
      for (uint32_t j = 1; j < segSB_weight[i].size(); ++j) 
      {
        if (segSB_weight[i][j].first % blk_per_superblk == parityBlkIdx) 
        {
          // check copydata calculate
          MGCcopydata = MGCcopydata + segSB_weight[i][j].second
            - MGCcandidate[i].second;

          MGCcandidate[i] = segSB_weight[i][j];
          break;
        }
      }

      
    }
    assert(MGCcandidate[i].first % blk_per_superblk == parityBlkIdx);
  }
#ifdef DEBUG
  cout << "\nGreedy\n";
  for(uint32_t i = 0; i < MGC_segments; ++i) {
    auto blk = blocks.find(MGCcandidate[i]);
    uint32_t pv = blk->second.getPartialValidPageCount(i);
    cout << pv << " ";
  }
  cout << "\n";
#endif
  // MLCgetGCvictimGreedy.c:187 twoSB
  // Inoder to restrict superblock (2) : MGCcandidate[0] & most repeated SB
  vector<uint32_t> sameSBcount(MGC_segments, 0);
  uint32_t mostSB = 0;
  maxCount = 0;
  mostSB = MGCcandidate[0].first;

  for (uint32_t i = 1; i < MGC_segments; ++i) 
  {
    if (MGCcandidate[i].first != MGCcandidate[0].first) 
    {
      sameSBcount[i]++;
      for (uint32_t j = 0; j < i; ++j) 
      {
        if (MGCcandidate[i].first == MGCcandidate[j].first) 
        {
          sameSBcount[j]++;
          break;
        }
      }
    }
  }
  assert(sameSBcount[0] == 0);

  // MLCgetGCvictimGreedy.c:207
  for (uint32_t i = 0; i < MGC_segments; ++i) 
  {
    if (sameSBcount[i] >= maxCount && MGCcandidate[i].first != MGCcandidate[0].first)
    {
      maxCount = sameSBcount[i];
      mostSB = MGCcandidate[i].first;
    }
  }


  // MLCgetGCvictimGreedy.c:215
  for (uint32_t i = 1; i < MGC_segments; ++i)
  {
    auto blk0 = blocks.find(MGCcandidate[0].first);
    auto blkSB = blocks.find(mostSB);

    uint32_t seg0_count = blk0->second.getPartialValidPageCount(i);
    uint32_t segSB_count = blkSB->second.getPartialValidPageCount(i);
    uint32_t seg_i_count = MGCcandidate[i].second;
    //  seg 1 ~ 3 choose less valid page segment between 0 and mostSB
    if (seg0_count <= segSB_count)
    {
      MGCcopydata = MGCcopydata + seg0_count - seg_i_count;
      MGCcandidate[i].first = MGCcandidate[0].first;
      MGCcandidate[i].second = seg0_count;
    }
    else
    {
      MGCcopydata = MGCcopydata + segSB_count - seg_i_count;
      MGCcandidate[i].first = mostSB;
      MGCcandidate[i].second = segSB_count;
    }
  }

  // MLCgetGCvictimGreedy.c:492
  uint32_t testsum = 0;
  for (int i = 0; i < MGC_segments; ++i)
    testsum += MGCcandidate[i].second;
  
  if (MGCcopydata != testsum) {
    cout << "MGCcopydata : " << MGCcopydata << "\n";
    cout << "testsum : " << testsum << "\n";
    panic("MGCcopydata mismatch");
  }
  
  // calculate rewrite parity
  for (uint32_t i = 1; i < MGC_segments; ++i) 
  {
    uint32_t j = 0;
    for (; j < i; ++j) 
    {
      if (MGCcandidate[i].first == MGCcandidate[j].first)
        break;
    }  
    if (j == i)
      MGCcopydata += 256;
  }

  uint32_t threshold = weight[0].second;

  if (threshold > MGCcopydata + 0) 
  {
    MGC = 1;
    list.emplace_back(MGCcandidate[0].first);
    stat.MGCcopydata += MGCcopydata;
  }
  else 
  {
    MGC = 0;
    list.emplace_back(weight[0].first);
  }

  if (MGC) 
  {
    // exchange segment to MGCcandidate[0]
    // cannot use move assignment (granularity different)
    for (uint32_t i = 1; i < MGC_segments; ++i) 
    {
      exchange_seg(MGCcandidate[0].first, MGCcandidate[i].first, i);
    }
    
  }

  

#else
  MGC = 0;
  for (uint64_t i = 0; i < 1; i++) {
    list.push_back(weight.at(i).first);
  }

#endif
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::SELECT_VICTIM_BLOCK);
}

void PageMapping::flushGCbuf(std::vector<PAL::Request> &writeRequests, uint64_t &tick, uint32_t SBtype) {

  PAL::Request pal_req(32);

  auto freeBlock = blocks.find(spareblk_idx);
  if (freeBlock->second.isfull()) {
    spareblk_idx = getFreeBlock(0);
    freeBlock = blocks.find(spareblk_idx);
  }

  if (freeBlock->second.getSBtype() == 9) {
    freeBlock->second.setSBtype(SBtype);
  }

  uint32_t newBlockIdx = freeBlock->first;

  auto &Buffer = (SBtype ? GCbuf_seg : GCbuf);

  for (auto &i : Buffer) { 
    uint64_t lpn = i.first;
    uint32_t valid = i.second;

    auto mappingList = table.find(lpn);

    if (mappingList == table.end()) {
      panic("Invalid mapping table entry");
    }

    if (freeBlock->second.isfull()) {
      spareblk_idx = getFreeBlock(0);
      freeBlock = blocks.find(spareblk_idx);
      freeBlock->second.setSBtype(SBtype);
    }
    newBlockIdx = freeBlock->first;

    // idx channel
    uint32_t write_channel_idx;
    if (SBtype == 1) {

#ifdef segmentGCbuffer
      write_channel_idx = freeBlock->second.getNextWriteChannelIndexSeg();
#else
      write_channel_idx = freeBlock->second.getNextWriteChannelIndex();
#endif

    }
    else {
      write_channel_idx = freeBlock->second.getNextWriteChannelIndex();
    }

    uint32_t parity_channel_idx = freeBlock->second.getParityChannelIndex();
    // parity
    while (write_channel_idx == parity_channel_idx) {

      // write to FTL
      uint32_t pageidx = freeBlock->second.getNextWritePageIndex(parity_channel_idx);
      freeBlock->second.write(pageidx, 0, parity_channel_idx, tick);
      freeBlock->second.invalidate(pageidx, parity_channel_idx);

      // PAL req
      PAL::Request parity_palreq(32);
      parity_palreq.blockIndex = freeBlock->first;
      parity_palreq.pageIndex = pageidx;
      parity_palreq.ioFlag.reset();
      parity_palreq.ioFlag.set(parity_channel_idx);

      // write to PAL
      writeRequests.emplace_back(parity_palreq);
      ++parity_write;
      ++pwrite[parity_channel_idx];

      // get next write 
      if (freeBlock->second.isfull()) {
        spareblk_idx = getFreeBlock(0);
        freeBlock = blocks.find(spareblk_idx);
        freeBlock->second.setSBtype(SBtype);
      }

      newBlockIdx = freeBlock->first;
      parity_channel_idx = freeBlock->second.getParityChannelIndex();

      if (SBtype == 1) {
#ifdef segmentGCbuffer
        write_channel_idx = freeBlock->second.getNextWriteChannelIndexSeg();
#else
        write_channel_idx = freeBlock->second.getNextWriteChannelIndex();
#endif

      }
      else if (SBtype == 0) {
        write_channel_idx = freeBlock->second.getNextWriteChannelIndex();
      }  
    }
    

    uint32_t newPageIdx = freeBlock->second.getNextWritePageIndex(write_channel_idx);
    auto &mapping = mappingList->second;

    freeBlock->second.write(newPageIdx, lpn, write_channel_idx, tick);

    // update mapping table
    if (valid) {
      lpn_channel[lpn] = write_channel_idx;
      mapping.first = newBlockIdx;
      mapping.second = newPageIdx;
    }
    else {
      freeBlock->second.invalidate(newPageIdx, write_channel_idx);
    }

    // Issue Write
    pal_req.blockIndex = newBlockIdx;
    pal_req.pageIndex = newPageIdx;
    pal_req.ioFlag.reset();
    pal_req.ioFlag.set(write_channel_idx);

    writeRequests.emplace_back(pal_req);
  }
  
  if (freeBlock->second.getParityChannelIndex() == 31
    && freeBlock->second.getNextWritePageIndex(31) == 255
    && freeBlock->second.getNextWritePageIndex(30) == 256) {

    uint32_t pageidx = freeBlock->second.getNextWritePageIndex(31);
    freeBlock->second.write(pageidx, 0, 31, tick);
    freeBlock->second.invalidate(pageidx, 31);

    // PAL req
    PAL::Request parity_palreq(32);
    parity_palreq.blockIndex = freeBlock->first;
    parity_palreq.pageIndex = pageidx;
    parity_palreq.ioFlag.reset();
    parity_palreq.ioFlag.set(31);
    writeRequests.emplace_back(parity_palreq);
    ++parity_write;
    ++pwrite[31];
    // get next write 
  }

  if (freeBlock->second.isfull()) {
    spareblk_idx = getFreeBlock(0);
  }

  if (SBtype == 1) {
    GCbuf_seg.clear();
#ifndef skip_invalidGCbuffer
    GCbuf_seg.reserve(GCbufSize);
#endif
  }
  else {
    GCbuf.clear();
#ifndef skip_invalidGCbuffer
    GCbuf.reserve(GCbufSize);
#endif
  }

#ifdef DEBUG
  if (!freeBlock->second.isempty() && !freeBlock->second.isfull()) {
    cout << "After flush GC buf" << "\n";
    getBlockStatus();
  }
#endif
}

void PageMapping::doGarbageCollection(std::vector<uint32_t> &blocksToReclaim,
                                      uint64_t &tick, uint32_t &MGC) {
  PAL::Request req(param.ioUnitInPage);
  std::vector<PAL::Request> readRequests;
  std::vector<PAL::Request> writeRequests;
  std::vector<PAL::Request> eraseRequests;
  std::vector<uint64_t> lpns;
  Bitset bit(param.ioUnitInPage);
  uint64_t beginAt;
  uint64_t readFinishedAt = tick;
  uint64_t writeFinishedAt = tick;
  uint64_t eraseFinishedAt = tick;

  if (blocksToReclaim.size() == 0) {
    return;
  }

#ifdef transposebuffer
  
  std::vector<vector<std::pair<uint64_t, int>>> transposeBuf;
  
  if (MGC) {

    transposeBuf.clear();
    transposeBuf = 
      std::vector<vector<std::pair<uint64_t, int>>> 
        (256, std::vector<std::pair<uint64_t, int>>(32, make_pair(0, 0)));

  }
  

#endif
  // For all blocks to reclaim, collecting request structure only
  for (auto &iter : blocksToReclaim) {
    auto block = blocks.find(iter);

    if (block == blocks.end()) {
      panic("Invalid block");
    }

    // Copy valid pages to free block
    for (uint32_t pageIndex = 0; pageIndex < param.pagesInBlock; pageIndex++) {
      // Valid?
      if (block->second.getPageInfo(pageIndex, lpns, bit)) {
        if (!bRandomTweak) {
          bit.set();
        }

        // Issue Read
        req.blockIndex = block->first;
        req.pageIndex = pageIndex;
        req.ioFlag = bit;

        readRequests.push_back(req);

        // Update mapping table
        //uint32_t newBlockIdx = freeBlock->first;

        for (uint32_t idx = 0; idx < bitsetSize; idx++) {
          if (bit.test(idx)) {
            
#ifdef GCbuffer
            // Invalidate
            block->second.invalidate(pageIndex, idx);

            if (MGC) {
              lpn_channel[lpns.at(idx)] = IN_TPBUFFER;
              transposeBuf.at(pageIndex).at(idx) = make_pair(lpns.at(idx), 1);
              continue;
            }
            else {

              lpn_channel[lpns.at(idx)] = IN_GCBUFFER;

            }
            
            GCbuf.emplace_back(lpns.at(idx), 1);

            // flush GCbuf
            if (GCbuf.size() == GCbufSize) {
              flushGCbuf(writeRequests, tick, 0);
            }

#else
            // Invalidate
            block->second.invalidate(pageIndex, idx);
            auto mappingList = table.find(lpns.at(idx));

            if (mappingList == table.end()) {
              panic("Invalid mapping table entry");
            }

            pDRAM->read(&(*mappingList), 8 * param.ioUnitInPage, tick);


            if (freeBlock->second.getNextWritePageIndex() == param.pagesInBlock) {
              spareblk_idx = getFreeBlock(0);
              freeBlock = blocks.find(spareblk_idx);
            }

            newBlockIdx = freeBlock->first;

            // idx channel
            uint32_t write_channel_idx = freeBlock->second.getNextWriteChannelIndex();
            uint32_t parity_channel_idx = freeBlock->second.getParityChannelIndex();
            // parity 
            if (!warmup && write_channel_idx == parity_channel_idx) {
              // FTL req
              Request parity_req(32);
              parity_req.lpn = 0;
              parity_req.ioFlag.reset();
              parity_req.ioFlag.set(parity_channel_idx);
              // might buggy, reqCount++
              parity_req.reqID = 0;
              parity_req.reqSubID = parity_channel_idx + 1;

              // write to FTL
              uint32_t pageIndex = freeBlock->second.getNextWritePageIndex(parity_channel_idx);
              freeBlock->second.write(pageIndex, parity_req.lpn, parity_channel_idx, tick);
              // no need to update mapping table
              // invalidate immediately
              freeBlock->second.invalidate(pageIndex, parity_channel_idx);

              // PAL req
              PAL::Request parity_palreq(parity_req);
              parity_palreq.blockIndex = freeBlock->first;
              parity_palreq.pageIndex = pageIndex;
              parity_palreq.ioFlag.reset();
              parity_palreq.ioFlag.set(parity_channel_idx);
              // write to PAL
              //pPAL->write(parity_palreq, tick);
              writeRequests.emplace_back(parity_palreq);

              ++parity_write;
              // get next write 
              if (freeBlock->second.getNextWritePageIndex() == param.pagesInBlock) {
                spareblk_idx = getFreeBlock(0);
                freeBlock = blocks.find(spareblk_idx);
              }
              write_channel_idx = freeBlock->second.getNextWriteChannelIndex();
            }
            // update mapping
            lpn_channel[lpns.at(idx)] = write_channel_idx;
            newBlockIdx = freeBlock->first;

            uint32_t newPageIdx = freeBlock->second.getNextWritePageIndex(write_channel_idx);
            auto &mapping = mappingList->second;

            mapping.first = newBlockIdx;
            mapping.second = newPageIdx;

            freeBlock->second.write(newPageIdx, lpns.at(idx), write_channel_idx, beginAt);

            // Issue Write
            req.blockIndex = newBlockIdx;
            req.pageIndex = newPageIdx;

            if (bRandomTweak) {
              req.ioFlag.reset();
              req.ioFlag.set(write_channel_idx);
            }
            else {
              req.ioFlag.set();
            }

            writeRequests.push_back(req);
#endif

            stat.validPageCopies++;
          }
        }

        stat.validSuperPageCopies++;
      }
    }

    // Erase block
    req.blockIndex = block->first;
    req.pageIndex = 0;
    req.ioFlag.set();

    eraseRequests.push_back(req);
  }

#ifdef transposebuffer

  if (MGC) {
    
  #ifdef segmentGCbuffer
    uint32_t segment_size = blk_per_superblk / MGC_segments;
    for (uint32_t i = 0; i < MGC_segments; ++i) {
      for (uint32_t j = 0; j < param.pagesInBlock; ++j) {
        for (uint32_t k = 0; k < segment_size; ++k) {

          if (transposeBuf[j][i * segment_size + k].second == 0)
            continue;
          
          GCbuf_seg.emplace_back(transposeBuf[j][i * segment_size + k].first, 1);

          lpn_channel[transposeBuf[j][i * segment_size + k].first] = IN_TPBUFFER;
          transposeBuf[j][i * segment_size + k].second = 0;

          if (GCbuf_seg.size() == GCbufSize) {
            flushGCbuf(writeRequests, tick, 1);
          }
        }
      }
    }
  #else
    for (uint32_t i = 0; i < param.pagesInBlock; ++i) {
      for (uint32_t j = 0; j < 32; ++j) {
        
        if (transposeBuf[i][j].second == 0)
          continue;
        
        GCbuf_seg.emplace_back(transposeBuf[i][j].first, 1);

        lpn_channel[transposeBuf[i][j].first] = IN_TPBUFFER;
        transposeBuf[i][j].second = 0;

        if (GCbuf_seg.size() == GCbufSize) {
          flushGCbuf(writeRequests, tick, 1);
        }
        
      }
    }
  
  #endif
  }
#endif
  // Do actual I/O here
  // This handles PAL2 limitation (SIGSEGV, infinite loop, or so-on)
  for (auto &iter : readRequests) {
    beginAt = tick;

    pPAL->read(iter, beginAt);

    readFinishedAt = MAX(readFinishedAt, beginAt);
  }

  for (auto &iter : writeRequests) {
    beginAt = readFinishedAt;

    pPAL->write(iter, beginAt);

    writeFinishedAt = MAX(writeFinishedAt, beginAt);
  }

  for (auto &iter : eraseRequests) {
    beginAt = readFinishedAt;

    eraseInternal(iter, beginAt);

    eraseFinishedAt = MAX(eraseFinishedAt, beginAt);
  }

  tick = MAX(tick, eraseFinishedAt);
  tick = MAX(tick, writeFinishedAt);
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::DO_GARBAGE_COLLECTION);
}


void PageMapping::readInternal(Request &req, uint64_t &tick) {
  PAL::Request palRequest(req);
  uint64_t beginAt;
  uint64_t finishedAt = tick;

  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
    }

    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      if (req.ioFlag.test(idx) || !bRandomTweak) {
        //auto &mapping = mappingList->second.at(idx);
        auto &mapping = mappingList->second;

        if (mapping.first < param.totalPhysicalBlocks &&
            mapping.second < param.pagesInBlock) {
          palRequest.blockIndex = mapping.first;
          palRequest.pageIndex = mapping.second;

          if (bRandomTweak) {
            palRequest.ioFlag.reset();
            palRequest.ioFlag.set(idx);
          }
          else {
            palRequest.ioFlag.set();
          }

          auto block = blocks.find(palRequest.blockIndex);

          if (block == blocks.end()) {
            panic("Block is not in use");
          }

          beginAt = tick;

          block->second.read(palRequest.pageIndex, idx, beginAt);
          pPAL->read(palRequest, beginAt);

          finishedAt = MAX(finishedAt, beginAt);
        }
      }
    }

    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ_INTERNAL);
  }
}

void PageMapping::writeInternal(Request &req, uint64_t &tick, bool sendToPAL, bool SBtype) {

  PAL::Request palRequest(req);
  std::unordered_map<uint32_t, Block>::iterator block;
  auto mappingList = table.find(req.lpn);
  uint64_t beginAt;
  uint64_t finishedAt = tick;
  bool readBeforeWrite = false;

  // GC 5 times
  if (nFreeBlocks <= 245) {
    while (nFreeBlocks <= 250) {
      if (!sendToPAL) {
        panic("ftl: GC triggered while in initialization");
      }

#ifdef DEBUG
      auto block = blocks.find(spareblk_idx);
    
      if (!block->second.isempty() && !block->second.isfull()) {
        cout << "Before GC\n";
        getBlockStatus();
      }
#endif    
  
      std::vector<uint32_t> list;
      uint64_t beginAt = tick;
      uint32_t MGC = 0;
      selectVictimBlock(list, beginAt, MGC);

      debugprint(LOG_FTL_PAGE_MAPPING,
                  "GC   | On-demand | %u blocks will be reclaimed", list.size());

      if (MGC)
        stat.MGCcount++;

      doGarbageCollection(list, beginAt, MGC);

      debugprint(LOG_FTL_PAGE_MAPPING,
                  "GC   | Done | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")", tick,
                  beginAt, beginAt - tick);

      stat.gcCount++;
      stat.reclaimedBlocks += list.size();

#ifdef DEBUG
      block = blocks.find(spareblk_idx);
      if (!block->second.isempty() && !block->second.isfull()) {
        cout << "After GC\n";
        getBlockStatus();
      }
#endif

    }
  }



  if (lpn_channel[req.lpn] == IN_GCBUFFER) {

#ifdef skip_invalidGCbuffer
    for (auto i = GCbuf.begin(); i != GCbuf.end(); ) {
      if (i->first == req.lpn) {
        i = GCbuf.erase(i);
      }
      else 
        ++i;
    }
#else
    for (auto &i : GCbuf) {
      if (i.first == req.lpn) {
        i.second = 0;
      }
    }
#endif

  }
  else if (lpn_channel[req.lpn] == IN_TPBUFFER) {

#ifdef skip_invalidGCbuffer
    for (auto i = GCbuf_seg.begin(); i != GCbuf_seg.end(); ) {
      if (i->first == req.lpn) {
        i = GCbuf_seg.erase(i);
      }
      else 
        ++i;
    }
#else
    for (auto &i : GCbuf_seg) {
      if (i.first == req.lpn) {
        i.second = 0;
      }
    }
#endif

  }
  else {

    if (mappingList != table.end()) {

      auto &mapping = mappingList->second;
      uint32_t invalid_idx = lpn_channel[req.lpn];
      if (mapping.first < param.totalPhysicalBlocks &&
          mapping.second < param.pagesInBlock && invalid_idx < 32) {
        block = blocks.find(mapping.first);

        // Invalidate current page
        block->second.invalidate(mapping.second, invalid_idx);
      }
        
      
    }
    else {
      // Create empty mapping
      auto ret = table.emplace(
          req.lpn,
          std::pair<uint32_t, uint32_t>{param.totalPhysicalBlocks, param.pagesInBlock});

      if (!ret.second) {
        panic("Failed to insert new mapping");
      }

      mappingList = ret.first;
    }

  }
  
  if (spareblk_idx == 9999999)
    spareblk_idx = getFreeBlock(0);
  
  // Write data to free block
  block = blocks.find(spareblk_idx);

  if (block->second.isfull()) {
    spareblk_idx = getFreeBlock(0);
    block = blocks.find(spareblk_idx);
  }

  uint32_t write_channel_idx;
  uint32_t parity_channel_idx = block->second.getParityChannelIndex();


  if (block->second.getSBtype() == 9) {
    block->second.setSBtype(SBtype);
  }

  if (SBtype == 1) {

#ifdef vertical
    write_channel_idx = block->second.getNextWriteChannelIndexSeg();
#else
    write_channel_idx = block->second.getNextWriteChannelIndexVertical();
#endif

  }
  else if (SBtype == 0) {
    write_channel_idx = block->second.getNextWriteChannelIndex();
  }


  // fill parity channel

  while (write_channel_idx == parity_channel_idx) {

    uint32_t pageIndex = block->second.getNextWritePageIndex(write_channel_idx);
    block->second.write(pageIndex, 0, write_channel_idx, tick);
    block->second.invalidate(pageIndex, write_channel_idx);
      // PAL req
    PAL::Request parity_palreq(32);
    parity_palreq.blockIndex = block->first;
    parity_palreq.pageIndex = pageIndex;
    parity_palreq.ioFlag.reset();
    parity_palreq.ioFlag.set(write_channel_idx);
    // write to PAL
    if (!warmup)
      pPAL->write(parity_palreq, tick);

    ++parity_write;
    ++pwrite[write_channel_idx];

    // get next write 
    if (block->second.isfull()) {
      spareblk_idx = getFreeBlock(0);
      block = blocks.find(spareblk_idx);
    }

    parity_channel_idx = block->second.getParityChannelIndex();

    if (SBtype == 1) {

#ifdef vertical
      write_channel_idx = block->second.getNextWriteChannelIndexSeg();
#else
      write_channel_idx = block->second.getNextWriteChannelIndexVertical();
#endif

    }
    else if (SBtype == 0)
      write_channel_idx = block->second.getNextWriteChannelIndex();
  

  }

  if (block->second.getSBtype() == 9) {
    block->second.setSBtype(SBtype);
  }

  req.ioFlag.reset();
  req.ioFlag.set(write_channel_idx);
  lpn_channel[req.lpn] = write_channel_idx;

  if (block == blocks.end()) {
    panic("No such block");
  }

  if (sendToPAL) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
      pDRAM->write(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
      pDRAM->write(&(*mappingList), 8, tick);
    }
  }

  if (!bRandomTweak && !req.ioFlag.all()) {
    // We have to read old data
    readBeforeWrite = true;
  }

  for (uint32_t idx = 0; idx < bitsetSize; idx++) {
    if (req.ioFlag.test(idx) || !bRandomTweak) {
      uint32_t pageIndex = block->second.getNextWritePageIndex(idx);
      //auto &mapping = mappingList->second.at(idx);
      auto &mapping = mappingList->second;

      beginAt = tick;
      block->second.write(pageIndex, req.lpn, idx, beginAt);

      // Read old data if needed (Only executed when bRandomTweak = false)
      // Maybe some other init procedures want to perform 'partial-write'
      // So check sendToPAL variable
      if (readBeforeWrite && sendToPAL) {
        palRequest.blockIndex = mapping.first;
        palRequest.pageIndex = mapping.second;

        // We don't need to read old data
        palRequest.ioFlag = req.ioFlag;
        palRequest.ioFlag.flip();

        pPAL->read(palRequest, beginAt);
      }

      // update mapping to table
      mapping.first = block->first;
      mapping.second = pageIndex;

      if (sendToPAL) {
        palRequest.blockIndex = block->first;
        palRequest.pageIndex = pageIndex;

        if (bRandomTweak) {
          palRequest.ioFlag.reset();
          palRequest.ioFlag.set(idx);
        }
        else {
          palRequest.ioFlag.set();
        }

        pPAL->write(palRequest, beginAt);
      }

      finishedAt = MAX(finishedAt, beginAt);
    }
  }


#ifndef vertical
  // fill parity condition
  if (SBtype == 1 && block->second.getParityChannelIndex() == 31 && block->second.getNextWritePageIndex(30) == 256) {
    
    bool fill_parity = 1;

    for (int i = 0; i < 31; ++i) {
      if (block->second.getNextWritePageIndex(i) != 256) {
        fill_parity = 0;
        break;
      }
    }

    if (fill_parity) {
      write_channel_idx = 31;
      uint32_t pageIndex = block->second.getNextWritePageIndex(parity_channel_idx);

      while (write_channel_idx == parity_channel_idx && pageIndex != 256) {
        block->second.write(pageIndex, 0, write_channel_idx, tick);
        block->second.invalidate(pageIndex, write_channel_idx);
          // PAL req
        PAL::Request parity_palreq(32);
        parity_palreq.blockIndex = block->first;
        parity_palreq.pageIndex = pageIndex;
        parity_palreq.ioFlag.reset();
        parity_palreq.ioFlag.set(write_channel_idx);
        // write to PAL
        if (!warmup)
          pPAL->write(parity_palreq, tick);

        ++parity_write;
        ++pwrite[parity_channel_idx];

        write_channel_idx = block->second.getNextWriteChannelIndexVertical();
        pageIndex = block->second.getNextWritePageIndex(write_channel_idx);

      }

    }
  }
#endif

  if (block->second.getParityChannelIndex()  == 31 
    && block->second.getNextWritePageIndex(31) == 255 && block->second.getNextWritePageIndex(30) == 256) {
    
    uint32_t pageIndex = block->second.getNextWritePageIndex(31);
    block->second.write(pageIndex, 0, 31, tick);
    block->second.invalidate(pageIndex, 31);

    // PAL req
    PAL::Request parity_palreq(32);
    parity_palreq.blockIndex = block->first;
    parity_palreq.pageIndex = pageIndex;
    parity_palreq.ioFlag.reset();
    parity_palreq.ioFlag.set(31);
    // write to PAL
    pPAL->write(parity_palreq, beginAt);
    ++parity_write;
    ++pwrite[31];

  }

  finishedAt = MAX(finishedAt, beginAt);

  // Exclude CPU operation when initializing
  if (sendToPAL) {
    tick = finishedAt;
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE_INTERNAL);
  }

  if (block->second.isfull()) {
    spareblk_idx = getFreeBlock(0);
  }

}

void PageMapping::trimInternal(Request &req, uint64_t &tick) {
  auto mappingList = table.find(req.lpn);

  if (mappingList != table.end()) {
    if (bRandomTweak) {
      pDRAM->read(&(*mappingList), 8 * req.ioFlag.count(), tick);
    }
    else {
      pDRAM->read(&(*mappingList), 8, tick);
    }

    // Do trim
    for (uint32_t idx = 0; idx < bitsetSize; idx++) {
      //auto &mapping = mappingList->second.at(idx);
      auto &mapping = mappingList->second;

      auto block = blocks.find(mapping.first);

      if (block == blocks.end()) {
        panic("Block is not in use");
      }

      block->second.invalidate(mapping.second, idx);
    }

    // Remove mapping
    table.erase(mappingList);

    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM_INTERNAL);
  }
}

void PageMapping::eraseInternal(PAL::Request &req, uint64_t &tick) {
  static uint64_t threshold =
      conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  auto block = blocks.find(req.blockIndex);

  // Sanity checks
  if (block == blocks.end()) {
    panic("No such block");
  }

  if (block->second.getValidPageCount() != 0) {
    panic("There are valid pages in victim block");
  }

  // Erase block
  block->second.erase();

  pPAL->erase(req, tick);

  // Check erase count
  uint32_t erasedCount = block->second.getEraseCount();

  if (erasedCount < threshold) {
    // Reverse search
    auto iter = freeBlocks.end();

    while (true) {
      iter--;

      if (iter->getEraseCount() <= erasedCount) {
        // emplace: insert before pos
        iter++;

        break;
      }

      if (iter == freeBlocks.begin()) {
        break;
      }
    }

    // Insert block to free block list
    freeBlocks.emplace(iter, std::move(block->second));
    nFreeBlocks++;
  }

  // Remove block from block list
  blocks.erase(block);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::ERASE_INTERNAL);
}

float PageMapping::calculateWearLeveling() {
  uint64_t totalEraseCnt = 0;
  uint64_t sumOfSquaredEraseCnt = 0;
  uint64_t numOfBlocks = param.totalLogicalBlocks;
  uint64_t eraseCnt;

  for (auto &iter : blocks) {
    eraseCnt = iter.second.getEraseCount();
    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  // freeBlocks is sorted
  // Calculate from backward, stop when eraseCnt is zero
  for (auto riter = freeBlocks.rbegin(); riter != freeBlocks.rend(); riter++) {
    eraseCnt = riter->getEraseCount();

    if (eraseCnt == 0) {
      break;
    }

    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  if (sumOfSquaredEraseCnt == 0) {
    return -1;  // no meaning of wear-leveling
  }

  return (float)totalEraseCnt * totalEraseCnt /
         (numOfBlocks * sumOfSquaredEraseCnt);
}

void PageMapping::calculateTotalPages(uint64_t &valid, uint64_t &invalid) {
  valid = 0;
  invalid = 0;

  for (auto &iter : blocks) {
    valid += iter.second.getValidPageCount();
    invalid += iter.second.getDirtyPageCount();
  }
}

void PageMapping::getStatList(std::vector<Stats> &list, std::string prefix) {
  Stats temp;

  temp.name = prefix + "page_mapping.gc.count";
  temp.desc = "Total GC count";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.reclaimed_blocks";
  temp.desc = "Total reclaimed blocks in GC";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.MGC.page_copies";
  temp.desc = "Total copied valid superpages during GC";
  list.push_back(temp);

  temp.name = prefix + "page_mapping.gc.page_copies";
  temp.desc = "Total copied valid pages during GC";
  list.push_back(temp);

  // For the exact definition, see following paper:
  // Li, Yongkun, Patrick PC Lee, and John Lui.
  // "Stochastic modeling of large-scale solid-state storage systems: analysis,
  // design tradeoffs and optimization." ACM SIGMETRICS (2013)
  temp.name = prefix + "page_mapping.MGC.count";
  temp.desc = "Wear-leveling factor";
  list.push_back(temp);
}

void PageMapping::getStatValues(std::vector<double> &values) {
  values.push_back(stat.gcCount - stat.MGCcount);
  values.push_back(stat.reclaimedBlocks);
  values.push_back(stat.MGCcopydata);
  values.push_back(stat.validPageCopies);
  values.push_back((float)stat.MGCcount);
}

void PageMapping::resetStatValues() {
  memset(&stat, 0, sizeof(stat));
}

}  // namespace FTL

}  // namespace SimpleSSD