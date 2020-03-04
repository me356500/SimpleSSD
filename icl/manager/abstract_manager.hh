// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_ICL_MANAGER_ABSTRACT_MANAGER_HH__
#define __SIMPLESSD_ICL_MANAGER_ABSTRACT_MANAGER_HH__

#include "icl/icl.hh"
#include "sim/object.hh"

namespace SimpleSSD::ICL {

class AbstractCache;

struct FlushContext {
  LPN lpn;           // Logical address of request
  uint64_t address;  // Physical address of internal DRAM
  uint8_t *buffer;   // Data (for simulation)

  FlushContext(LPN l, uint64_t a) : lpn(l), address(a) {}
};

class SequentialDetector {
 protected:
  bool enabled;

  uint32_t pageSize;

 public:
  SequentialDetector(uint32_t p) : enabled(false), pageSize(p) {}

  inline bool isEnabled() { return enabled; }

  virtual void submitSubRequest(HIL::SubRequest *) = 0;
};

class AbstractManager : public Object {
 protected:
  ICL::ICL *pICL;

  FTL::FTL *pFTL;
  AbstractCache *cache;

  Event eventICLCompletion;

  inline HIL::SubRequest *getSubRequest(uint64_t tag) {
    return pICL->getSubRequest(tag);
  }

 public:
  AbstractManager(ObjectData &o, ICL::ICL *p, FTL::FTL *f)
      : Object(o),
        pICL(p),
        pFTL(f),
        cache(nullptr),
        eventICLCompletion(InvalidEventID) {}
  virtual ~AbstractManager() {}

  void setCallbackFunction(Event e) { eventICLCompletion = e; }
  virtual void initialize(AbstractCache *ac) { cache = ac; }

  /* Interface for ICL::ICL */
  //! Submit read request
  virtual void read(HIL::SubRequest *) = 0;

  //! Submit write request
  virtual void write(HIL::SubRequest *) = 0;

  //! Submit flush request
  virtual void flush(HIL::SubRequest *) = 0;

  //! Submit trim/format request (erase data in cache)
  virtual void erase(HIL::SubRequest *) = 0;

  //! Called by ICL when DMA is completed (for releasing cacheline)
  virtual void dmaDone(HIL::SubRequest *) = 0;

  /* Interface for ICL::AbstractCache */
  /**
   * \brief Handler for cache lookup
   *
   * Called when cacheline lockup has been completed
   *
   * \param[in] tag Tag of SubRequest
   */
  virtual void lookupDone(uint64_t tag) = 0;

  //! Completion handler for other cache jobs
  virtual void cacheDone(uint64_t tag) = 0;

  //! Cache write-back requester
  virtual void drain(std::vector<FlushContext> &) = 0;
};

}  // namespace SimpleSSD::ICL

#endif
