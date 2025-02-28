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

#include "ftl/common/block.hh"

#include <algorithm>
#include <cstring>

namespace SimpleSSD {

namespace FTL {

Block::Block(uint32_t blockIdx, uint32_t count, uint32_t ioUnit)
    : idx(blockIdx),
      pageCount(count),
      ioUnitInPage(ioUnit),
      pValidBits(nullptr),
      pErasedBits(nullptr),
      pLPNs(nullptr),
      ppLPNs(nullptr),
      lastAccessed(0),
      eraseCount(0) {
  if (ioUnitInPage == 1) {
    pValidBits = new Bitset(pageCount);
    pErasedBits = new Bitset(pageCount);

    pLPNs = (uint64_t *)calloc(pageCount, sizeof(uint64_t));
  }
  else if (ioUnitInPage > 1) {
    Bitset copy(ioUnitInPage);

    validBits = std::vector<Bitset>(pageCount, copy);
    erasedBits = std::vector<Bitset>(pageCount, copy);

    ppLPNs = (uint64_t **)calloc(pageCount, sizeof(uint64_t *));

    for (uint32_t i = 0; i < pageCount; i++) {
      ppLPNs[i] = (uint64_t *)calloc(ioUnitInPage, sizeof(uint64_t));
    }
  }
  else {
    panic("Invalid I/O unit in page");
  }

  // C-style allocation
  pNextWritePageIndex = (uint32_t *)calloc(ioUnitInPage, sizeof(uint32_t));
  erase();
  eraseCount = 0;
  write_channel_idx = 0;
  SBtype = 9;
  write_seg_idx = 0;
  partialvalidBits = std::vector<uint32_t>(4, 0);
  validpage_cnt = 0;
}

Block::Block(const Block &old)
    : Block(old.idx, old.pageCount, old.ioUnitInPage) {
  if (ioUnitInPage == 1) {
    *pValidBits = *old.pValidBits;
    *pErasedBits = *old.pErasedBits;

    memcpy(pLPNs, old.pLPNs, pageCount * sizeof(uint64_t));
  }
  else {
    validBits = old.validBits;
    erasedBits = old.erasedBits;

    for (uint32_t i = 0; i < pageCount; i++) {
      memcpy(ppLPNs[i], old.ppLPNs[i], ioUnitInPage * sizeof(uint64_t));
    }
  }

  memcpy(pNextWritePageIndex, old.pNextWritePageIndex,
         ioUnitInPage * sizeof(uint32_t));
         
  eraseCount = old.eraseCount;
  write_channel_idx = old.write_channel_idx;
  SBtype = old.SBtype;
  write_seg_idx = old.write_seg_idx;
  partialvalidBits = old.partialvalidBits;
  validpage_cnt = old.validpage_cnt;
}

Block::Block(Block &&old) noexcept
    : idx(std::move(old.idx)),
      pageCount(std::move(old.pageCount)),
      ioUnitInPage(std::move(old.ioUnitInPage)),
      pNextWritePageIndex(std::move(old.pNextWritePageIndex)),
      pValidBits(std::move(old.pValidBits)),
      pErasedBits(std::move(old.pErasedBits)),
      pLPNs(std::move(old.pLPNs)),
      validBits(std::move(old.validBits)),
      erasedBits(std::move(old.erasedBits)),
      ppLPNs(std::move(old.ppLPNs)),
      lastAccessed(std::move(old.lastAccessed)),
      eraseCount(std::move(old.eraseCount)),
      write_channel_idx(std::move(old.write_channel_idx)),
      SBtype(std::move(old.SBtype)),
      write_seg_idx(std::move(old.write_seg_idx)),
      partialvalidBits(std::move(old.partialvalidBits)),
      validpage_cnt(std::move(old.validpage_cnt)) {
  // TODO Use std::exchange to set old value to null (C++14)
  old.idx = 0;
  old.pageCount = 0;
  old.ioUnitInPage = 0;
  old.pNextWritePageIndex = nullptr;
  old.pValidBits = nullptr;
  old.pErasedBits = nullptr;
  old.pLPNs = nullptr;
  old.ppLPNs = nullptr;
  old.lastAccessed = 0;
  old.eraseCount = 0;
  old.write_channel_idx = 0;
  old.SBtype = 9;
  old.write_seg_idx = 0;
  old.partialvalidBits = std::vector<uint32_t>(4, 0);
  old.validpage_cnt = 0;
}

Block::~Block() {
  free(pNextWritePageIndex);

  if (pLPNs)
    free(pLPNs);

  delete pValidBits;
  delete pErasedBits;

  if (ppLPNs) {
    for (uint32_t i = 0; i < pageCount; i++) {
      free(ppLPNs[i]);
    }

    free(ppLPNs);
  }

  pNextWritePageIndex = nullptr;
  pLPNs = nullptr;
  pValidBits = nullptr;
  pErasedBits = nullptr;
  ppLPNs = nullptr;
}

Block &Block::operator=(const Block &rhs) {
  if (this != &rhs) {
    this->~Block();
    *this = Block(rhs);  // Call copy constructor
  }

  return *this;
}

Block &Block::operator=(Block &&rhs) {
  if (this != &rhs) {
    this->~Block();

    idx = std::move(rhs.idx);
    pageCount = std::move(rhs.pageCount);
    ioUnitInPage = std::move(rhs.ioUnitInPage);
    pNextWritePageIndex = std::move(rhs.pNextWritePageIndex);
    pValidBits = std::move(rhs.pValidBits);
    pErasedBits = std::move(rhs.pErasedBits);
    pLPNs = std::move(rhs.pLPNs);
    validBits = std::move(rhs.validBits);
    erasedBits = std::move(rhs.erasedBits);
    ppLPNs = std::move(rhs.ppLPNs);
    lastAccessed = std::move(rhs.lastAccessed);
    eraseCount = std::move(rhs.eraseCount);
    write_channel_idx = std::move(rhs.write_channel_idx);
    SBtype = std::move(rhs.SBtype);
    write_seg_idx = std::move(rhs.write_seg_idx);
    partialvalidBits = std::move(rhs.partialvalidBits);
    validpage_cnt = std::move(rhs.validpage_cnt);

    rhs.pNextWritePageIndex = nullptr;
    rhs.pValidBits = nullptr;
    rhs.pErasedBits = nullptr;
    rhs.pLPNs = nullptr;
    rhs.ppLPNs = nullptr;
    rhs.lastAccessed = 0;
    rhs.eraseCount = 0;
    rhs.write_channel_idx = 0;
    rhs.SBtype = 9;
    rhs.write_seg_idx = 0;
    rhs.partialvalidBits = std::vector<uint32_t>(4, 0);
    rhs.validpage_cnt = 0;
  }

  return *this;
}

uint32_t Block::getBlockIndex() const {
  return idx;
}

uint32_t Block::getParityChannelIndex() const {
  return idx % 32;
}

uint64_t Block::getLastAccessedTime() {
  return lastAccessed;
}

uint32_t Block::getEraseCount() {
  return eraseCount;
}

uint32_t Block::getBlockPageCount(uint32_t channel) {
  uint32_t ret = 0;
  // ioUnitInpage == 32
  if (ioUnitInPage == 1) {
    panic("ioUnitInPage error");
  }

  // pageindex, channel index
  for(uint32_t i = 0; i < 256; ++i)
  {
    ret += validBits[i].test(channel);
  }

  return ret;
}

uint32_t Block::getStripePageCount(uint32_t idx) {
  uint32_t ret = 0;

  // pageindex, channel index
  for(uint32_t i = 0; i < 32; ++i)
  {
    ret += validBits[idx].test(i);
  }

  return ret;
}

uint32_t Block::getPartialValidPageCount(uint32_t idx) {
  uint32_t ret = 0;

  // 256 pages 0~63 64~127 128~191 192~255
  if(ioUnitInPage == 1)
  {
    for(uint32_t pageIndex = 64 * idx; pageIndex < 64 * (idx + 1); ++pageIndex) 
    {
      uint8_t data = pValidBits->getData(pageIndex);
      if(data & (0x01 << (pageIndex % 8))) 
      {
        ret++;
      }
    }
  }
  else
  {
    return partialvalidBits.at(idx);
  }
  
  return ret;
}

uint32_t Block::getValidPageCount() {
  uint32_t ret = 0;

  if (ioUnitInPage == 1) {
    ret = pValidBits->count();
  }
  else {
    for (auto &iter : validBits) {
      if (iter.any()) {
        ret++;
      }
    }
  }

  return ret;
}

uint32_t Block::getValidPageCountRaw() {
  uint32_t ret = 0;

  if (ioUnitInPage == 1) {
    // Same as getValidPageCount()
    ret = pValidBits->count();
  }
  else {
    return validpage_cnt;
  }

  return ret;
}

uint32_t Block::getDirtyPageCount() {
  uint32_t ret = 0;

  if (ioUnitInPage == 1) {
    ret = (~(*pValidBits | *pErasedBits)).count();
  }
  else {
    for (uint32_t i = 0; i < pageCount; i++) {
      // Dirty: Valid(false), Erased(false)
      if ((~(validBits.at(i) | erasedBits.at(i))).any()) {
        ret++;
      }
    }
  }

  return ret;
}

uint32_t Block::getNextWritePageIndex() {
  uint32_t idx = 0;

  for (uint32_t i = 0; i < ioUnitInPage; i++) {
    if (idx < pNextWritePageIndex[i]) {
      idx = pNextWritePageIndex[i];
    }
  }

  return idx;
}

uint32_t Block::isfull() {
  bool full = 1;

  for (uint32_t i = 0; i < ioUnitInPage; ++i) {
    if (pNextWritePageIndex[i] != 256) {
      full = 0;
      break;
    }
  }

  return full;
}

uint32_t Block::isempty() {
  bool empty = 1;
  
  for (uint32_t i = 0; i < ioUnitInPage; ++i) {
    if (pNextWritePageIndex[i] != 0) {
      empty = 0;
      break;
    }
  }

  return empty;
}

uint32_t Block::getNextWritePageIndex(uint32_t idx) {
  return pNextWritePageIndex[idx];
}

bool Block::getPageInfo(uint32_t pageIndex, std::vector<uint64_t> &lpn,
                        Bitset &map) {
  if (ioUnitInPage == 1 && map.size() == 1) {
    map.set();
    lpn = std::vector<uint64_t>(1, pLPNs[pageIndex]);
  }
  else if (map.size() == ioUnitInPage) {
    map = validBits.at(pageIndex);
    lpn = std::vector<uint64_t>(32);
    for (uint32_t idx = 0; idx < 32; ++idx) {
      lpn[idx] = ppLPNs[pageIndex][idx];
    }

  }
  else {
    panic("I/O map size mismatch");
  }

  return map.any();
}

uint64_t *Block::getpLPN() {
  return pLPNs;
}

bool Block::getValidBits(uint32_t pageIndex, uint32_t idx) {
  return validBits.at(pageIndex).test(idx);
}

bool Block::getErasedBits(uint32_t pageIndex, uint32_t idx) {
  return erasedBits.at(pageIndex).test(idx);
}

uint64_t Block::getppLPN(uint32_t pageIndex, uint32_t idx) {
  return ppLPNs[pageIndex][idx];
}

bool Block::read(uint32_t pageIndex, uint32_t idx, uint64_t tick) {
  bool read = false;

  if (ioUnitInPage == 1 && idx == 0) {
    read = pValidBits->test(pageIndex);
  }
  else if (idx < ioUnitInPage) {
    read = validBits.at(pageIndex).test(idx);
  }
  else {
    panic("I/O map size mismatch");
  }

  if (read) {
    if (tick == (uint64_t)(1e63))
      return read;
    //lastAccessed = tick;
  }

  return read;
}

bool Block::write(uint32_t pageIndex, uint64_t lpn, uint32_t idx,
                  uint64_t tick) {
  bool write = false;

  if (ioUnitInPage == 1 && idx == 0) {
    write = pErasedBits->test(pageIndex);
  }
  else if (idx < ioUnitInPage) {
    write = erasedBits.at(pageIndex).test(idx);
  }
  else {
    panic("I/O map size mismatch");
  }

  if (write) {
    if (pageIndex < pNextWritePageIndex[idx]) {
      panic("Write to block should sequential");
    }

    lastAccessed = tick;

    if (ioUnitInPage == 1) {
      pErasedBits->reset(pageIndex);
      pValidBits->set(pageIndex);

      pLPNs[pageIndex] = lpn;
    }
    else {
      erasedBits.at(pageIndex).reset(idx);
      validBits.at(pageIndex).set(idx);

      ppLPNs[pageIndex][idx] = lpn;
    }
    // update partial valid
    /*
      0 ~ 7   : 0
      8 ~ 15  : 1
      16 ~ 23 : 2
      24 ~ 31 : 3
    */
    uint32_t segment_idx = idx / 8;
    partialvalidBits[segment_idx]++;
    validpage_cnt++;

    pNextWritePageIndex[idx] = pageIndex + 1;
  }
  else {
    panic("Write to non erased page");
  }

  return write;
}

void Block::erase() {
  if (ioUnitInPage == 1) {
    pValidBits->reset();
    pErasedBits->set();
  }
  else {
    for (auto &iter : validBits) {
      iter.reset();
    }
    for (auto &iter : erasedBits) {
      iter.set();
    }
  }

  memset(pNextWritePageIndex, 0, sizeof(uint32_t) * ioUnitInPage);
  write_channel_idx = 0;
  SBtype = 9;
  write_seg_idx = 0;
  partialvalidBits = std::vector<uint32_t>(4, 0);
  validpage_cnt = 0;
  eraseCount++;
}

void Block::invalidate(uint32_t pageIndex, uint32_t idx) {
  if (ioUnitInPage == 1) {
    pValidBits->reset(pageIndex);
  }
  else {
    validBits.at(pageIndex).reset(idx);
    uint32_t segment_idx = idx / 8;
    partialvalidBits[segment_idx]--;
    validpage_cnt--;
  }
}

void Block::setValidBits(uint32_t pageIndex, uint32_t idx, bool value) {

  bool origin = validBits.at(pageIndex).test(idx);

  if (!origin && value) {
    validpage_cnt++;
    uint32_t segment_idx = idx / 8;
    partialvalidBits[segment_idx]++;
  }

  if (origin && !value) {
    validpage_cnt--;
    uint32_t segment_idx = idx / 8;
    partialvalidBits[segment_idx]--;
  }

  validBits.at(pageIndex).set(idx, value);

}

void Block::setErasedBits(uint32_t pageIndex, uint32_t idx, bool value) {
  erasedBits.at(pageIndex).set(idx, value);
}

void Block::setppLPN(uint32_t pageIndex, uint32_t idx, uint64_t lpn) {
  ppLPNs[pageIndex][idx] = lpn;
}

uint32_t Block::getNextWriteChannelIndex() {
  write_channel_idx %= 32;
  return write_channel_idx++;
}

uint32_t Block::getNextWriteChannelIndexVertical() {

  if (pNextWritePageIndex[write_channel_idx] == 256) {
    ++write_channel_idx;
    
  }
  write_channel_idx %= 32;
  return write_channel_idx;
}

uint32_t Block::getNextWriteChannelIndexSeg() {
  uint32_t end_ch = 8 * (write_seg_idx + 1) - 1;
  
  if (pNextWritePageIndex[end_ch] == 256) {
    write_seg_idx++;
    write_channel_idx = write_seg_idx * 8;
  }
  else if (write_channel_idx > end_ch) {
    write_channel_idx = write_seg_idx * 8;
  }


  return write_channel_idx++;
}

uint32_t Block::getSBtype() {
  return SBtype;
}

void Block::setSBtype(uint32_t type) {
  SBtype = type;
}

}  // namespace FTL

}  // namespace SimpleSSD