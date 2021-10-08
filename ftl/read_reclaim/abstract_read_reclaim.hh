// SPDX-License-Identifier: LGPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_FTL_READ_RECLAIM_ABSTRACT_READ_RECLAIM_HH__
#define __SIMPLESSD_FTL_READ_RECLAIM_ABSTRACT_READ_RECLAIM_HH__

#include <random>

#include "fil/fil.hh"
#include "ftl/allocator/victim_selection.hh"
#include "ftl/background_manager/abstract_background_job.hh"
#include "ftl/def.hh"

namespace SimpleSSD::FTL::ReadReclaim {

enum class State : uint32_t {
  Idle,

  Foreground,  // Invoked by reported RBER
  Background,
};

class AbstractReadReclaim : public AbstractBlockCopyJob {
 protected:
  State state;

  std::default_random_engine engine;

  uint32_t estimateBitError(uint64_t now, const PSBN &psbn);

  uint32_t getParallelBlockCount() override { return 1; }
  std::string getPrefix() override { return "FTL::BasicReadReclaim"; }
  const char *getLogPrefix() override { return "RR    "; }

 public:
  AbstractReadReclaim(ObjectData &, FTLObjectData &, FIL::FIL *);
  virtual ~AbstractReadReclaim();

  void initialize() override;
  bool isRunning() override;

  void triggerByUser(TriggerType, Request *) override;

  /* Read Reclaim APIs */

  virtual bool doErrorCheck(const PPN &);

  void createCheckpoint(std::ostream &) const noexcept override;
  void restoreCheckpoint(std::istream &) noexcept override;
};

}  // namespace SimpleSSD::FTL::ReadReclaim

#endif
