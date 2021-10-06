// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2019 CAMELab
 *
 * Author: Donghyun Gouk <kukdh1@camelab.org>
 */

#pragma once

#ifndef __SIMPLESSD_FTL_JOB_MANAGER_HH__
#define __SIMPLESSD_FTL_JOB_MANAGER_HH__

#include "ftl/def.hh"
#include "ftl/object.hh"
#include "sim/object.hh"

namespace SimpleSSD::FTL {

class AbstractJob : public Object {
 protected:
  FTLObjectData &ftlobject;

 public:
  AbstractJob(ObjectData &o, FTLObjectData &fo) : Object(o), ftlobject(fo) {}
  virtual ~AbstractJob() {}

  virtual void initialize() {}

  virtual bool trigger_readMapping(Request *) { return false; }
  virtual bool trigger_readSubmit(Request *) { return false; }
  virtual bool trigger_readDone(Request *) { return false; }
  virtual bool trigger_writeMapping(Request *) { return false; }
  virtual bool trigger_writeSubmit(Request *) { return false; }
  virtual bool trigger_writeDone(Request *) { return false; }
};

class JobManager : public Object {
 protected:
  std::vector<AbstractJob *> jobs;

 public:
  JobManager(ObjectData &);
  ~JobManager();

  /**
   * \brief Add FTL job to job manager
   *
   * This function must call in constructor of FTL.
   */
  void addJob(AbstractJob *);

  /**
   * \brief Initialize all jobs
   */
  void initialize();

  bool trigger_readMapping(Request *);
  bool trigger_readSubmit(Request *);
  bool trigger_readDone(Request *);
  bool trigger_writeMapping(Request *);
  bool trigger_writeSubmit(Request *);
  bool trigger_writeDone(Request *);

  void getStatList(std::vector<Stat> &, std::string) noexcept override;
  void getStatValues(std::vector<double> &) noexcept override;
  void resetStatValues() noexcept override;

  void createCheckpoint(std::ostream &) const noexcept override;
  void restoreCheckpoint(std::istream &) noexcept override;
};

}  // namespace SimpleSSD::FTL

#endif
