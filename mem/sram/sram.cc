// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "mem/sram/sram.hh"

namespace SimpleSSD::Memory::SRAM {

SRAM::SRAM(ObjectData &o)
    : AbstractSRAM(o),
      scheduler(
          o, "Memory::SRAM::scheduler",
          [this](Request *r) -> uint64_t { return preSubmit(r); },
          [this](Request *r) -> uint64_t { return preSubmit(r); },
          [this](Request *r) { postDone(r); },
          [this](Request *r) { postDone(r); },
          [this](std::ostream &o, Request *r) { Request::backup(o, r); },
          [this](std::istream &i) -> Request * {
            return Request::restore(i);
          }) {
  // Convert cycle to ps
  pStructure->latency =
      (uint64_t)(pStructure->latency * 1000000000000.f /
                 readConfigUint(Section::CPU, CPU::Config::Clock));
}

SRAM::~SRAM() {}

uint64_t SRAM::preSubmit(Request *req) {
  // Calculate latency
  return DIVCEIL(req->length, pStructure->lineSize) * pStructure->latency;
}

void SRAM::postDone(Request *req) {
  // Call handler
  schedule(req->eid, getTick());

  delete req;
}

void SRAM::read(uint64_t address, uint64_t length, Event eid) {
  auto req = new Request(address, length, eid);

  rangeCheck(address, length);

  // Enqueue request
  req->beginAt = getTick();

  readStat.count++;
  readStat.size += length;

  scheduler.read(req);
}

void SRAM::write(uint64_t address, uint64_t length, Event eid) {
  auto req = new Request(address, length, eid);

  rangeCheck(address, length);

  // Enqueue request
  req->beginAt = getTick();

  writeStat.count++;
  writeStat.size += length;

  scheduler.write(req);
}

void SRAM::createCheckpoint(std::ostream &out) const noexcept {
  AbstractSRAM::createCheckpoint(out);

  scheduler.createCheckpoint(out);
}

void SRAM::restoreCheckpoint(std::istream &in) noexcept {
  AbstractSRAM::restoreCheckpoint(in);

  scheduler.restoreCheckpoint(in);
}

}  // namespace SimpleSSD::Memory::SRAM
