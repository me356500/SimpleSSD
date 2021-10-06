// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_FTL_ALLOCATOR_VICTIM_SELECTION_HH__
#define __SIMPLESSD_FTL_ALLOCATOR_VICTIM_SELECTION_HH__

#include "ftl/object.hh"
#include "sim/object.hh"

namespace SimpleSSD::FTL::BlockAllocator {

class AbstractAllocator;

class AbstractVictimSelection {
 protected:
  AbstractAllocator *pAllocator;

 public:
  AbstractVictimSelection(AbstractAllocator *);
  virtual ~AbstractVictimSelection();

  /**
   * \brief Select victim block
   *
   * \param idx   Parallelism index.
   * \param psbn  Physical superblock number.
   * \return Firmware execution information.
   */
  virtual CPU::Function getVictim(uint32_t idx, PSBN &psbn) noexcept = 0;
};

enum class VictimSelectionID {
  /* GC must uses following four algorithms */

  Random,       // Select victim block randomly in full block pool.
  Greedy,       // Select block with a largest number of invalid blocks.
  CostBenefit,  // Cost-benefit victim block selection algorithm.
  DChoice,      // D-Choice victim block selection algorithm.

  /* Below functions may return invalid PSBN */

  LeastErased,        // Select block with smallest P/E cycle.
  LeastRead,          // Select block with smallest read count after erase.
  MostErased,         // Select block with largest P/E cycle.
  MostRead,           // Select block with largest read count after erase.
  LeastRecentlyUsed,  // Select least recently accessed block after erase.
  MostRecentlyUsed,   // Select most recently accessed block after erase.
};

/**
 * \brief Get the Victim Selection Algorithm object
 *
 * This function returns victim selection algorithm object.
 * See comments of each object for more details.
 *
 * \param id Algorithm ID.
 * \return Victim selection algorithm.
 */
AbstractVictimSelection *GetVictimSelectionAlgorithm(VictimSelectionID id);

}  // namespace SimpleSSD::FTL::BlockAllocator

#endif
