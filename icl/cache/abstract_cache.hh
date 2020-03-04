// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_ICL_CACHE_ABSTRACT_CACHE_HH__
#define __SIMPLESSD_ICL_CACHE_ABSTRACT_CACHE_HH__

#include "icl/icl.hh"
#include "sim/object.hh"

namespace SimpleSSD::ICL {

class AbstractManager;

struct CacheLine {
  union {
    uint8_t data;
    struct {
      uint8_t valid : 1;       //!< Entry is valid
      uint8_t dirty : 1;       //!< Entry is dirty - should be written back
      uint8_t nvmPending : 1;  //!< Currently in reading/writing to NVM
      uint8_t dmaPending : 1;  //!< Currently in reading/writing to Host
      uint8_t rsvd : 4;
    };
  };

  LPN tag;              //!< LPN address of this cacheline
  uint64_t insertedAt;  //!< Inserted time
  uint64_t accessedAt;  //!< Created time
  Bitset validbits;     //!< Valid sector bits

  CacheLine(uint64_t size)
      : data(0), tag(0), insertedAt(0), accessedAt(0), validbits(size) {}
};

class AbstractCache : public Object {
 protected:
  static const uint64_t minIO = 512;

  AbstractManager *manager;
  FTL::Parameter *parameter;

 public:
  AbstractCache(ObjectData &o, AbstractManager *m, FTL::Parameter *p)
      : Object(o), manager(m), parameter(p) {}
  virtual ~AbstractCache() {}

  /**
   * \brief Lookup cache
   *
   * Set sreq->setAllocate() when cache needs new cacheline for current
   * subrequest. Call manager->lookupDone when completed.
   *
   * \param[in] sreq    Current subrequest
   */
  virtual void lookup(HIL::SubRequest *sreq) = 0;

  /**
   * \brief Flush cacheline
   *
   * Flush cacheline in [offset, offset + length) range. Call manager->cacheDone
   * when completed. Use manager->drain for data write-back.
   *
   * \param[in] sreq  Current subrequest
   */
  virtual void flush(HIL::SubRequest *sreq) = 0;

  /**
   * \brief Erase cacheline
   *
   * Erase (invalidate) cacheline in [offset, offset + length) range. Call
   * manager->cacheDone when completed.
   *
   * \param[in] sreq  Current subrequest
   */
  virtual void erase(HIL::SubRequest *sreq) = 0;

  /**
   * \brief Allocate cacheline in cache
   *
   * When miss, cache should make place to store new data. Call
   * manager->cacheDone when completed. Use manager->drain for data write-back.
   *
   * \param[in] sreq  Current subrequest
   */
  virtual void allocate(HIL::SubRequest *sreq) = 0;

  /**
   * \brief DMA done callback
   *
   * Called when DMA operation (Host <-> DRAM) has been completed.
   *
   * \param[in] lpn LPN address than completed
   */
  virtual void dmaDone(LPN lpn) = 0;

  /**
   * \brief NVM done callback
   *
   * Called when DMA operation (DRAM <-> NVM) has been completed.
   *
   * \param[in] lpn LPN address than completed
   */
  virtual void nvmDone(LPN lpn) = 0;
};

}  // namespace SimpleSSD::ICL

#endif
