// See LICENSE for license details.

#ifndef _RISCV_SIMIF_H
#define _RISCV_SIMIF_H

#include <map>
#include "decode.h"
#include "cfg.h"
#include "abstract_interrupt_controller.h"

#ifdef FORCE_RISCV_ENABLE
#include "Force_Memory.h"
#endif

class processor_t;
class mmu_t;

// this is the interface to the simulator used by the processors and memory
class simif_t
{
public:
#ifndef FORCE_RISCV_ENABLE
  // should return NULL for MMIO addresses
  virtual char* addr_to_mem(reg_t paddr) = 0;
  virtual bool reservable(reg_t paddr) { return addr_to_mem(paddr); }
  // used for MMIO addresses
  virtual bool mmio_fetch(reg_t paddr, size_t len, uint8_t* bytes) { return mmio_load(paddr, len, bytes); }
  virtual bool mmio_load(reg_t paddr, size_t len, uint8_t* bytes) = 0;
  virtual bool mmio_store(reg_t paddr, size_t len, const uint8_t* bytes) = 0;
  // Callback for processors to let the simulation know they were reset.
  virtual void proc_reset(unsigned id) = 0;

  // abtract to reset simulator
  virtual void reset() = 0;

  virtual const cfg_t &get_cfg() const = 0;
  virtual const std::map<size_t, processor_t*>& get_harts() const = 0;

  virtual abstract_interrupt_controller_t* get_intctrl() const = 0;

  virtual const char* get_symbol(uint64_t paddr) = 0;

  virtual ~simif_t() = default;

  unsigned nprocs() const { return get_cfg().nprocs(); }
#else
  // should return NULL for MMIO addresses
  virtual char* addr_to_mem(reg_t paddr) = 0;
  // virtual bool is_initialized(reg_t paddr) = 0;
  // virtual bool is_reserved(reg_t paddr) = 0;
  virtual bool reservable(reg_t paddr) { return addr_to_mem(paddr); }
  // used for MMIO addresses
  virtual bool mmio_fetch(reg_t paddr, size_t len, uint8_t* bytes) { return mmio_load(paddr, len, bytes); }
  virtual bool mmio_load(reg_t paddr, size_t len, uint8_t* bytes) = 0;
  virtual bool mmio_store(reg_t paddr, size_t len, const uint8_t* bytes) = 0;
  // Callback for processors to let the simulation know they were reset.
  virtual void proc_reset(unsigned id) = 0;

  // abtract to reset simulator
  virtual void reset() = 0;

  virtual const cfg_t &get_cfg() const = 0;
  virtual const std::map<size_t, processor_t*>& get_harts() const = 0;

  virtual abstract_interrupt_controller_t* get_intctrl() const = 0;

  virtual const char* get_symbol(uint64_t paddr) = 0;

  virtual ~simif_t() = default;

  unsigned nprocs() const { return get_cfg().nprocs(); }

  // to support sparse memory model
  virtual uint64_t sparse_read(reg_t paddr, size_t len) = 0;
  virtual void sparse_read_partially_initialized(reg_t paddr, size_t len, uint8_t* bytes) = 0;
  virtual void sparse_write(reg_t paddr, const uint8_t* bytes, size_t len) = 0;
  virtual void sparse_write(reg_t paddr, uint64_t value, size_t len) = 0;
  virtual void sparse_write_with_initialization(reg_t paddr, const uint8_t *bytes, size_t len) = 0;
  virtual bool sparse_is_pa_initialized(reg_t paddr, size_t len) = 0;
  virtual void sparse_initialize_pa(reg_t paddr, reg_t value, size_t numBytes, Force::EMemDataType type) = 0;
  virtual void sparse_initialize_pa(reg_t paddr, const uint8_t* data, const uint8_t* attrs, uint32_t nBytes, Force::EMemDataType type) = 0;
  virtual void sparse_reserve(reg_t paddr, size_t numBytes) = 0;
  virtual void sparse_unreserve(reg_t paddr, size_t numBytes) = 0;
  virtual bool sparse_is_reserved(reg_t paddr, size_t numBytes) = 0;
#endif

  static const size_t INTERLEAVE = 5000;
  static const size_t INSNS_PER_RTC_TICK = 100; // 10 MHz clock for 1 BIPS core
  static const size_t CPU_HZ = 1000000000; // 1GHz CPU

  mmu_t* debug_mmu;  // debug port into main memory, for use by debug_module
};

#endif
