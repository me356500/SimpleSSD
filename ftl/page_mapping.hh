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

#ifndef __FTL_PAGE_MAPPING__
#define __FTL_PAGE_MAPPING__

#include <cinttypes>
#include <unordered_map>
#include <vector>

#include "ftl/abstract_ftl.hh"
#include "ftl/common/block.hh"
#include "ftl/ftl.hh"
#include "pal/pal.hh"
#include <sys/time.h>

#define parity 1
#define MGC_segments 4
#define blk_per_superblk 32
#define GCbufSize 7936
#define writeBufSize 7936
#define IN_GCBUFFER 7777
#define IN_TPBUFFER 6666
#define writebuffer
#define GCbuffer
#define vertical
//#define DEBUG
#define transposebuffer
namespace SimpleSSD {

namespace FTL {

class PageMapping : public AbstractFTL {
 private:
  PAL::PAL *pPAL;

  ConfigReader &conf;

  //std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>>
  //    table;
  std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>>
      table;
  std::unordered_map<uint32_t, Block> blocks;
  std::list<Block> freeBlocks;
  uint32_t nFreeBlocks;  // For some libraries which std::list::size() is O(n)
  std::vector<uint32_t> lastFreeBlock;
  Bitset lastFreeBlockIOMap;
  uint32_t lastFreeBlockIndex;

  bool bReclaimMore;
  bool bRandomTweak;
  uint32_t bitsetSize;

  std::vector<Request> writeBufVertical;
  //std::vector<Request> GCbuf;
  // only need lpn
  std::list<uint64_t> GCbuf;
  //std::vector<PAL::Request> GCbuf;
  std::vector<Request> writeBuf;
  double segSB_time;
  vector<vector<pair<uint32_t, uint32_t>>> segSB_weight;
  uint64_t parity_cnt;
  uint64_t cur_tick;
  uint64_t GCcopypage;
  uint32_t warmup;
  vector<uint32_t> pwrite;
  uint32_t parity_write;
  //std::unordered_map<uint64_t, uint32_t> lpn_channel;
  vector<uint32_t> lpn_channel;
  uint32_t spareblk_idx;
  uint32_t MGCcount;
  struct {
    uint64_t gcCount;
    uint64_t reclaimedBlocks;
    uint64_t validSuperPageCopies;
    uint64_t validPageCopies;
  } stat;

  float freeBlockRatio();
  uint32_t convertBlockIdx(uint32_t);
  uint32_t getFreeBlock(uint32_t);
  uint32_t getLastFreeBlock(Bitset &);
  void calculateVictimWeight(std::vector<std::pair<uint32_t, float>> &,
                             const EVICT_POLICY, uint64_t);
  void exchange_seg(uint32_t &, uint32_t &, uint32_t );
  void selectVictimBlock(std::vector<uint32_t> &, uint64_t &, uint32_t &);
  
  void flushGCbuf(std::vector<PAL::Request> &, uint64_t &);
  void doGarbageCollection(std::vector<uint32_t> &, uint64_t &, uint32_t &);

  void write_parity(uint32_t &, uint64_t &, uint32_t &);

  float calculateWearLeveling();
  void calculateTotalPages(uint64_t &, uint64_t &);

  void readInternal(Request &, uint64_t &);
  void writeInternal(Request &, uint64_t &, bool = true, bool = false);
  void trimInternal(Request &, uint64_t &);
  void eraseInternal(PAL::Request &, uint64_t &);

  void getBlockStatus();
 public:
  PageMapping(ConfigReader &, Parameter &, PAL::PAL *, DRAM::AbstractDRAM *);
  ~PageMapping();

  bool initialize() override;

  void read(Request &, uint64_t &) override;
  void write(Request &, uint64_t &, bool = false) override;
  void trim(Request &, uint64_t &) override;

  void format(LPNRange &, uint64_t &) override;

  Status *getStatus(uint64_t, uint64_t) override;

  void getStatList(std::vector<Stats> &, std::string) override;
  void getStatValues(std::vector<double> &) override;
  void resetStatValues() override;
};

}  // namespace FTL

}  // namespace SimpleSSD

#endif
