// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_FTL_BASE_BASIC_FTL_HH__
#define __SIMPLESSD_FTL_BASE_BASIC_FTL_HH__

#include "ftl/base/abstract_ftl.hh"

namespace SimpleSSD::FTL {

class BasicFTL : public AbstractFTL {
 protected:
  uint32_t pageSize;

  bool mergeReadModifyWrite;

  // Statistics
  struct {
    uint64_t count;
    uint64_t blocks;
    uint64_t superpages;
    uint64_t pages;
  } stat;

  virtual inline void triggerGC() {
    if (pAllocator->checkGCThreshold()) {
      scheduleNow(eventGCTrigger);
    }
  }

  Event eventReadSubmit;
  void read_submit(uint64_t);

  Event eventReadDone;
  void read_done(uint64_t);

  Event eventWriteSubmit;
  void write_submit(uint64_t);

  Event eventWriteDone;
  void write_done(uint64_t);

  Event eventInvalidateSubmit;
  void invalidate_submit(uint64_t, uint64_t);

  Event eventGCTrigger;
  void gc_trigger(uint64_t);

 public:
  BasicFTL(ObjectData &, FTL *, FIL::FIL *, Mapping::AbstractMapping *,
           BlockAllocator::AbstractAllocator *);
  virtual ~BasicFTL();

  void read(Request *) override;
  void write(Request *) override;
  void invalidate(Request *) override;

  void getStatList(std::vector<Stat> &, std::string) noexcept override;
  void getStatValues(std::vector<double> &) noexcept override;
  void resetStatValues() noexcept override;

  void createCheckpoint(std::ostream &) const noexcept override;
  void restoreCheckpoint(std::istream &) noexcept override;
};

}  // namespace SimpleSSD::FTL

#endif
