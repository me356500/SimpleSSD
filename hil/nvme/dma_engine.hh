// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __HIL_NVME_DMA_ENGINE_HH__
#define __HIL_NVME_DMA_ENGINE_HH__

#include <vector>

#include "hil/common/dma_engine.hh"

namespace SimpleSSD::HIL::NVMe {

class PRPEngine : public DMAEngine {
 private:
  class PRP {
   public:
    uint64_t address;
    uint64_t size;

    PRP();
    PRP(uint64_t, uint64_t);
  };

  class PRPInitContext {
   public:
    uint64_t handledSize;
    uint8_t *buffer;
    uint64_t bufferSize;
    Event eid;
    void *context;
  };

  bool inited;

  std::vector<PRP> prpList;
  uint64_t totalSize;
  uint64_t pageSize;

  Event readPRPList;

  uint64_t getSizeFromPRP(uint64_t);
  void getPRPListFromPRP(uint64_t, Event, void *);
  void getPRPListFromPRP_readDone(uint64_t, PRPInitContext *s);

 public:
  PRPEngine(ObjectData &&, Interface *, uint64_t);
  ~PRPEngine();

  void initData(uint64_t, uint64_t, uint64_t, Event, void * = nullptr);
  void initQueue(uint64_t, uint64_t, bool, Event, void * = nullptr);

  void read(uint64_t, uint64_t, uint8_t *, Event, void * = nullptr) override;
  void write(uint64_t, uint64_t, uint8_t *, Event, void * = nullptr) override;
};

class SGLEngine : public DMAEngine {};

}  // namespace SimpleSSD::HIL::NVMe

#endif
