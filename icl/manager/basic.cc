// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "icl/manager/basic.hh"

#include "icl/cache/abstract_cache.hh"

namespace SimpleSSD::ICL {

BasicCache::BasicCache(ObjectData &o, FTL::FTL *f) : AbstractManager(o, f) {
  // Create events
  eventLookupDone =
      createEvent([this](uint64_t t, uint64_t d) { lookupDone(t, d); },
                  "ICL::BasicCache::eventLookupDone");
}

BasicCache::~BasicCache() {
  if (!requestQueue.empty()) {
    warn("Request queue is not empty.");
  }
}

void BasicCache::lookupDone(uint64_t now, uint64_t tag) {
  auto iter = requestQueue.find(tag);

  panic_if(iter == requestQueue.end(), "Unexpected SubRequest ID %" PRIu64 ".",
           tag);

  auto &req = *iter->second;

  if (req.getHit()) {
    scheduleNow(eventICLCompletion, tag);

    requestQueue.erase(iter);
  }

  // If not hit, just wait for allocateDone()
}

void BasicCache::read(SubRequest *req) {
  bool hit = false;
  LPN lpn = req->getLPN();

  requestQueue.emplace(req->getTag(), req);

  // Lookup
  auto fstat = cache->lookup(lpn, false, hit);

  if (hit) {
    // Cache hit, immediate completion
    req->setHit();
  }
  else {
    fstat += cache->allocate(lpn, req->getTag());

    // Continue at allocateDone
  }

  scheduleFunction(CPU::CPUGroup::InternalCache, eventLookupDone, req->getTag(),
                   fstat);
}

void BasicCache::write(SubRequest *req) {
  bool hit = false;
  LPN lpn = req->getLPN();

  requestQueue.emplace(req->getTag(), req);

  // Lookup
  auto fstat = cache->lookup(lpn, true, hit);

  if (hit) {
    // Hit or cold-miss
    req->setHit();
  }
  else {
    fstat += cache->allocate(lpn, req->getTag());

    // Continue at allocateDone
  }

  scheduleFunction(CPU::CPUGroup::InternalCache, eventLookupDone, req->getTag(),
                   fstat);
}

void BasicCache::flush(SubRequest *req) {
  auto fstat = cache->flush(req->getOffset(), req->getLength());

  requestQueue.emplace(req->getTag(), req);

  // Continue at flushDone

  // TODO: Handle CPU
}

void BasicCache::erase(SubRequest *req) {
  auto fstat = cache->erase(req->getOffset(), req->getLength());

  requestQueue.emplace(req->getTag(), req);

  // Continue at eraseDone

  // TODO: Handle CPU
}

void BasicCache::dmaDone(SubRequest *req) {
  cache->dmaDone(req->getLPN());
}

void BasicCache::drain(std::vector<FlushContext> &list) {
  // TODO: FTL
}

void BasicCache::getStatList(std::vector<Stat> &, std::string) noexcept {}

void BasicCache::getStatValues(std::vector<double> &) noexcept {}

void BasicCache::resetStatValues() noexcept {}

void BasicCache::createCheckpoint(std::ostream &) const noexcept {}

void BasicCache::restoreCheckpoint(std::istream &) noexcept {}

}  // namespace SimpleSSD::ICL
