// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#include "hil/nvme/command/write.hh"

#include "hil/nvme/command/internal.hh"
#include "util/disk.hh"

namespace SimpleSSD::HIL::NVMe {

Write::Write(ObjectData &o, Subsystem *s) : Command(o, s) {
  dmaInitEvent =
      createEvent([this](uint64_t, uint64_t gcid) { dmaInitDone(gcid); },
                  "HIL::NVMe::Write::dmaInitEvent");
  dmaCompleteEvent =
      createEvent([this](uint64_t, uint64_t gcid) { dmaComplete(gcid); },
                  "HIL::NVMe::Write::dmaCompleteEvent");
  writeDoneEvent =
      createEvent([this](uint64_t, uint64_t gcid) { writeDone(gcid); },
                  "HIL::NVMe::Write::readDoneEvent");
}

void Write::dmaInitDone(uint64_t gcid) {
  auto tag = findDMATag(gcid);
  auto pHIL = subsystem->getHIL();
  auto &scmd = pHIL->getCommandManager()->getSubCommand(gcid).front();

  scmd.status = Status::DMA;

  // Perform first page DMA
  tag->dmaEngine->read(tag->dmaTag, 0, scmd.buffer.size() - scmd.skipFront,
                       scmd.buffer.data() + scmd.skipFront, dmaCompleteEvent,
                       gcid);
}

void Write::dmaComplete(uint64_t gcid) {
  auto tag = findDMATag(gcid);
  auto pHIL = subsystem->getHIL();
  auto &cmd = pHIL->getCommandManager()->getCommand(gcid);
  auto nslist = subsystem->getNamespaceList();
  auto ns = nslist.find(tag->sqc->getData()->namespaceID);
  auto disk = ns->second->getDisk();

  // Find dma subcommand
  uint32_t i = 0;
  uint32_t scmds = cmd.subCommandList.size();

  for (i = 0; i < scmds; i++) {
    auto &iter = cmd.subCommandList.at(i);

    if (iter.status == Status::DMA) {
      pHIL->submitSubcommand(gcid, i);

      // Handle disk
      if (disk) {
        disk->write(i * iter.buffer.size() + iter.skipFront,
                    iter.buffer.size() - iter.skipEnd,
                    iter.buffer.data() + iter.skipFront);
      }

      break;
    }
  }

  // Start next DMA
  if (LIKELY(i < scmds)) {
    auto &scmd = cmd.subCommandList.at(i);

    scmd.status = Status::DMA;

    tag->dmaEngine->read(
        tag->dmaTag,
        i * scmd.buffer.size() - cmd.subCommandList.front().skipFront,
        scmd.buffer.size() - scmd.skipEnd, scmd.buffer.data(), dmaCompleteEvent,
        gcid);
  }
}

void Write::writeDone(uint64_t gcid) {
  auto tag = findDMATag(gcid);
  auto pHIL = subsystem->getHIL();
  auto mgr = pHIL->getCommandManager();
  auto &cmd = mgr->getCommand(gcid);
  uint32_t completed = 0;

  for (auto &iter : cmd.subCommandList) {
    if (iter.status == Status::Done) {
      completed++;
    }
  }

  if (completed == cmd.subCommandList.size()) {
    auto now = getTick();

    debugprint_command(tag,
                       "NVM     | Write | NSID %u | %" PRIx64 "h + %" PRIx64
                       "h | %" PRIu64 " - %" PRIu64 " (%" PRIu64 ")",
                       tag->sqc->getData()->namespaceID, tag->_slba, tag->_nlb,
                       tag->beginAt, now, now - tag->beginAt);

    mgr->destroyCommand(gcid);
    subsystem->complete(tag);
  }
}

void Write::setRequest(ControllerData *cdata, SQContext *req) {
  auto tag = createDMATag(cdata, req);
  auto entry = req->getData();

  // Get parameters
  uint32_t nsid = entry->namespaceID;
  uint64_t slba = ((uint64_t)entry->dword11 << 32) | entry->dword10;
  uint16_t nlb = (entry->dword12 & 0xFFFF) + 1;
  // uint8_t dtype = (entry->dword12 >> 20) & 0x0F;
  // bool fua = entry->dword12 & 0x40000000;
  // bool lr = entry->dword12 & 0x80000000;
  // uint16_t dspec = entry->dword13 >> 16;
  // uint8_t dsm = entry->dword13 & 0xFF;

  debugprint_command(tag,
                     "NVM     | Write | NSID %u | %" PRIx64 "h + %" PRIx64 "h",
                     nsid, slba, nlb);

  // Make response
  tag->createResponse();

  // Check namespace
  auto nslist = subsystem->getNamespaceList();
  auto ns = nslist.find(nsid);

  if (UNLIKELY(ns == nslist.end())) {
    tag->cqc->makeStatus(true, false, StatusType::GenericCommandStatus,
                         GenericCommandStatusCode::Invalid_Field);

    subsystem->complete(tag);

    return;
  }

  // Convert unit
  LPN slpn;
  uint32_t nlp;
  uint32_t skipFront;
  uint32_t skipEnd;

  ns->second->getConvertFunction()(slba, nlb, slpn, nlp, &skipFront, &skipEnd);

  // Check range
  auto info = ns->second->getInfo();
  auto range = info->namespaceRange;

  if (UNLIKELY(slpn + nlp > range.second)) {
    tag->cqc->makeStatus(true, false, StatusType::GenericCommandStatus,
                         GenericCommandStatusCode::Invalid_Field);

    subsystem->complete(tag);

    return;
  }

  slpn += info->namespaceRange.first;

  ns->second->write(nlb * info->lbaSize);

  // Make command
  auto disk = ns->second->getDisk();
  auto pHIL = subsystem->getHIL();
  auto mgr = pHIL->getCommandManager();
  auto gcid = tag->getGCID();
  auto &cmd = mgr->createCommand(gcid, writeDoneEvent);

  cmd.opcode = Operation::Write;
  cmd.offset = slpn;
  cmd.length = nlp;

  for (LPN i = slpn; i < slpn + nlp; i++) {
    auto &scmd = mgr->createSubCommand(gcid);

    scmd.lpn = i;

    if (i == slpn) {
      scmd.skipFront = skipFront;
    }
    else if (i + 1 == slpn + nlp) {
      scmd.skipEnd = skipEnd;
    }

    scmd.buffer.resize(info->lpnSize);
  }

  tag->_slba = slba;
  tag->_nlb = nlb;
  tag->beginAt = getTick();

  tag->createDMAEngine(nlp * info->lpnSize - skipFront - skipEnd, dmaInitEvent);
}

void Write::getStatList(std::vector<Stat> &, std::string) noexcept {}

void Write::getStatValues(std::vector<double> &) noexcept {}

void Write::resetStatValues() noexcept {}

void Write::createCheckpoint(std::ostream &out) const noexcept {
  Command::createCheckpoint(out);

  BACKUP_EVENT(out, dmaInitEvent);
  BACKUP_EVENT(out, writeDoneEvent);
  BACKUP_EVENT(out, dmaCompleteEvent);
}

void Write::restoreCheckpoint(std::istream &in) noexcept {
  Command::restoreCheckpoint(in);

  RESTORE_EVENT(in, dmaInitEvent);
  RESTORE_EVENT(in, writeDoneEvent);
  RESTORE_EVENT(in, dmaCompleteEvent);
}

}  // namespace SimpleSSD::HIL::NVMe
