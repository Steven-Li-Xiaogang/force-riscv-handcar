// See LICENSE for license details.

#ifndef _RISCV_SIM_H
#define _RISCV_SIM_H

#ifndef FORCE_RISCV_ENABLE
#include "cfg.h"
#include "debug_module.h"
#include "devices.h"
#include "log_file.h"
#include "processor.h"
#include "simif.h"

#include <fesvr/memif.h>
#include <fesvr/htif.h>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <sys/types.h>

#else
#include "config.h"
#include "cfg.h"
#include "devices.h"
#include "log_file.h"
#include "processor.h"
#include "simif.h"

#include <fesvr/memif.h>
#include <fesvr/htif.h>

#include <vector>
#include <string>
#include <memory>
#include <sys/types.h>

#include "Force_Enums.h"
#include "Force_Memory.h"
#endif

class mmu_t;
#ifndef FORCE_RISCV_ENABLE
class remote_bitbang_t;
class socketif_t;
#endif

// this class encapsulates the processors and memory in a RISC-V machine.
#ifndef FORCE_RISCV_ENABLE
class sim_t : public htif_t, public simif_t
{
public:
  sim_t(const cfg_t *cfg, bool halted,
        std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
        std::vector<device_factory_t*> plugin_device_factories,
        const std::vector<std::string>& args,
        const debug_module_config_t &dm_config, const char *commit_log_file,
        bool dtb_enabled, const char *dtb_file,
        bool socket_enabled,
        FILE *cmd_file); // needed for command line option --cmd
  ~sim_t();

  // run the simulation to completion
  int run();
  void set_debug(bool value);
  void set_histogram(bool value);
  void add_device(reg_t addr, std::shared_ptr<abstract_device_t> dev);

  // Configure logging
  //
  // If enable_log is true, an instruction trace will be generated. If
  // enable_commitlog is true, so will the commit results
  void configure_log(bool enable_log, bool enable_commitlog);

  void set_procs_debug(bool value);
  void set_remote_bitbang(remote_bitbang_t* remote_bitbang) {
    this->remote_bitbang = remote_bitbang;
  }
  const char* get_dts() { return dts.c_str(); }
  processor_t* get_core(size_t i) { return procs.at(i); }
  virtual abstract_interrupt_controller_t* get_intctrl() const override { assert(plic.get()); return plic.get(); }
  virtual const cfg_t &get_cfg() const override { return *cfg; }

  virtual const std::map<size_t, processor_t*>& get_harts() const override { return harts; }

  // Callback for processors to let the simulation know they were reset.
  virtual void proc_reset(unsigned id) override;

private:
  isa_parser_t isa;
  const cfg_t * const cfg;
  std::vector<std::pair<reg_t, abstract_mem_t*>> mems;
  std::vector<processor_t*> procs;
  std::map<size_t, processor_t*> harts;
  std::pair<reg_t, reg_t> initrd_range;
  std::string dts;
  std::string dtb;
  bool dtb_enabled;
  std::vector<std::shared_ptr<abstract_device_t>> devices;
  std::shared_ptr<clint_t> clint;
  std::shared_ptr<plic_t> plic;
  bus_t bus;
  log_file_t log_file; // commit log output file

  FILE *cmd_file; // pointer to debug command input file

  socketif_t *socketif;
  std::ostream sout_; // used for socket and terminal interface

  processor_t* get_core(const std::string& i);
  void step(size_t n); // step through simulation
  size_t current_step;
  size_t current_proc;
  bool debug;
  bool histogram_enabled; // provide a histogram of PCs
  bool log;
  remote_bitbang_t* remote_bitbang;
  std::optional<std::function<void()>> next_interactive_action;

  // memory-mapped I/O routines
  virtual char* addr_to_mem(reg_t paddr) override;
  virtual bool mmio_load(reg_t paddr, size_t len, uint8_t* bytes) override;
  virtual bool mmio_store(reg_t paddr, size_t len, const uint8_t* bytes) override;
  void set_rom();

  virtual const char* get_symbol(uint64_t paddr) override;

  // presents a prompt for introspection into the simulation
  void interactive();

  // functions that help implement interactive()
  void interactive_help(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_quit(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_run(const std::string& cmd, const std::vector<std::string>& args, bool noisy);
  void interactive_run_noisy(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_run_silent(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_vreg(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_reg(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_freg(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_fregh(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_fregs(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_fregd(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_pc(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_priv(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_mem(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_str(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_dumpmems(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_mtime(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_mtimecmp(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_until(const std::string& cmd, const std::vector<std::string>& args, bool noisy);
  void interactive_until_silent(const std::string& cmd, const std::vector<std::string>& args);
  void interactive_until_noisy(const std::string& cmd, const std::vector<std::string>& args);
  reg_t get_reg(const std::vector<std::string>& args);
  freg_t get_freg(const std::vector<std::string>& args, int size);
  reg_t get_mem(const std::vector<std::string>& args);
  reg_t get_pc(const std::vector<std::string>& args);

  friend class processor_t;
  friend class mmu_t;

  // htif
  virtual void reset() override;
  virtual void idle() override;
  virtual void read_chunk(addr_t taddr, size_t len, void* dst) override;
  virtual void write_chunk(addr_t taddr, size_t len, const void* src) override;
  virtual size_t chunk_align() override { return 8; }
  virtual size_t chunk_max_size() override { return 8; }
  virtual endianness_t get_target_endianness() const override;

public:
  // Initialize this after procs, because in debug_module_t::reset() we
  // enumerate processors, which segfaults if procs hasn't been initialized
  // yet.
  debug_module_t debug_module;
};

#else
/* cosim doesnt have htif */
// class simlib_t : public simif_t
class simlib_t : public htif_t, public simif_t
{
public:
  // cfg_t: isa, priv, varch, nprocs, start_pc, hartids
  simlib_t(const cfg_t *cfg, bool halted, bool auto_init_mem,
        std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
        std::vector<device_factory_t*> plugin_device_factories,
        const std::vector<std::string>& args,
        const char *commit_log_file, FILE *cmd_file); // needed for command line option --cmd
  ~simlib_t();

  // load the elf file and reset
  int load_program_now(const char* elfPath);

  // run the simulation incrementally
  int step_simulator(int target_id, int num_steps, int stx_failed);

  // fetch the instruction at the given pc using debug_mmu and return the opcode and disassembly
  int get_disassembly(int target_id, const uint64_t* pc, char** opcode, char** disassembly);

  // run the simulation to completion
  void set_debug(bool debug);
  void set_log(bool enable_log, bool enable_commit_log);
  void set_histogram(bool histogram);

  void set_procs_debug(bool proc_debug);

  // const char* get_dts() { return dts.c_str(); }
  // processor_t* get_core(size_t i) { return procs.at(i); }

  processor_t* get_core(size_t i) {
    for (processor_t* proc_ptr : procs) {
      if (proc_ptr != nullptr && proc_ptr->get_state() != nullptr && proc_ptr->get_state()->pid == i)
        return proc_ptr;
    }

    return nullptr;
  }

  virtual abstract_interrupt_controller_t* get_intctrl() const override { assert(plic.get()); return plic.get(); }
  virtual const cfg_t &get_cfg() const override { return *cfg; }

  virtual const std::map<size_t, processor_t*>& get_harts() const override { return harts; }

  // Callback for processors to let the simulation know they were reset.
  virtual void proc_reset(unsigned id) override;

  // check core id existed
  bool core_id_exist(size_t i) {
    for (processor_t* proc_ptr : procs) {
      if (proc_ptr != nullptr && proc_ptr->get_state() != nullptr && proc_ptr->get_state()->pid == i)
        return true;
    }

    return false;
  }

  void dump_sparse_memory(std::ostream& out);

  reg_t get_entry_point() { return entry; };

  uint64_t get_csr_number(const std::string& input_name);
  uint64_t get_xpr_number(const std::string& input_name);
  uint64_t get_fpr_number(const std::string& input_name);
  uint64_t get_vecr_number(const std::string& input_name);

  std::string get_csr_name(uint64_t index);
  std::string get_xpr_name(uint64_t index);
  std::string get_fpr_name(uint64_t index);
  std::string get_vecr_name(uint64_t index);

  int read_csr(uint32_t procid, const std::string& input_name, uint64_t* value, uint32_t* length);
  int read_csr(uint32_t procid, uint64_t index, uint64_t* value, uint32_t* length);

  int read_xpr(uint32_t procid, const std::string& input_name, uint64_t* value, uint32_t* length);
  int read_xpr(uint32_t procid, uint64_t index, uint64_t* value, uint32_t* length);

  int read_fpr(uint32_t procid, const std::string& input_name, uint8_t* value, uint32_t* length);
  int read_fpr(uint32_t procid, uint64_t index, uint8_t* value, uint32_t* length);

  int read_vecr(uint32_t procid, const std::string& input_name, uint8_t* value, uint32_t* length);
  int read_vecr(uint32_t procid, uint64_t index, uint8_t* value, uint32_t* length);
  int partial_read_vecr(uint32_t procid, uint64_t index, uint8_t* pValue, uint32_t length, uint32_t offset);

  int write_csr(uint32_t procid, const std::string& input_name, const uint64_t* value, uint32_t length);
  int write_csr(uint32_t procid, uint64_t index, const uint64_t* value, uint32_t length);

  int write_xpr(uint32_t procid, const std::string& input_name, const uint64_t* value, uint32_t length);
  int write_xpr(uint32_t procid, uint64_t index, const uint64_t* value, uint32_t length);

  int write_fpr(uint32_t procid, const std::string& input_name, const uint8_t* value, uint32_t length);
  int write_fpr(uint32_t procid, uint64_t index, const uint8_t* value, uint32_t length);

  int write_vecr(uint32_t procid, const std::string& input_name, const uint8_t* value, uint32_t length);
  int write_vecr(uint32_t procid, uint64_t index, const uint8_t* value, uint32_t length);
  int partial_write_vecr(uint32_t procid, uint64_t index, const uint8_t* pValue, uint32_t length, uint32_t offset);

  uint64_t sparse_read(reg_t paddr, size_t len);
  void sparse_read_partially_initialized(reg_t paddr, size_t len, uint8_t* bytes);
  void sparse_write(reg_t paddr, uint64_t value, size_t len);
  void sparse_write(reg_t paddr, const uint8_t* bytes, size_t len);
  void sparse_write_with_initialization(reg_t paddr, const uint8_t* bytes, size_t len);
  void sparse_initialize_pa(reg_t paddr, reg_t value, size_t numBytes);
  void sparse_initialize_pa(reg_t paddr, const uint8_t* data, const uint8_t *attrs, uint32_t nBytes, Force::EMemDataType type);
  void sparse_initialize_pa(reg_t paddr, reg_t value, size_t numBytes, Force::EMemDataType type);
  bool sparse_is_pa_initialized(reg_t paddr, size_t len);
  void sparse_reserve(reg_t paddr, size_t numBytes) override;
  void sparse_unreserve(reg_t paddr, size_t numBytes) override;
  bool sparse_is_reserved(reg_t paddr, size_t numBytes) override;
  void initialize_multiword(reg_t taddr, size_t len, const void* src); // To support multiword initializations during elf loading

  bool set_pc_api(int procid, const std::string& name, const uint8_t* bytes, size_t len);
  bool get_pc_api(int procid, const std::string& name, uint8_t* bytes, size_t len);

  bool set_privilege_api(int procid, const uint64_t* val);
  bool get_privilege_api(int procid, uint64_t* val);

  // translate_virtual_address_api function: attempts to translate a virtual address into a physical address, returns any error information and also gathers the relevant pmp address and pmp configuration.
  //
  //  meaning of 'intent':
  //    0 - indicates a 'LOAD' access
  //    1 - indicates a 'STORE' access
  //    2 - indicates a 'FETCH' access
  //
  //  returns:
  //    0 - success
  //    1 - some pointer arguments were null
  //    2 - invalid procid
  //    3 - PMP problem with PA after address translation somehow
  //    4 - access exception while trying to check pmp status of page table entry PA
  //    5 - walk was unsuccessful and access type was FETCH
  //    6 - walk was unsuccessful and access type was LOAD
  //    7 - walk was unsuccessful and access type was STORE
  //    8 - walk was unsuccessful and access type was not any of the above
  //
  #define SIM_WALK_SUCCESS 0
  #define SIM_WALK_ARGS_POINTER_NULL 1
  #define SIM_WALK_INVALID_PROCID 2
  #define SIM_WALK_EXCEPTION_PA_OF_PMP 3
  #define SIM_WALK_EXCEPTION_PTE_PA_OF_PMP 4
  #define SIM_WALK_UNSUCCESSFUL_FETCH 5
  #define SIM_WALK_UNSUCCESSFUL_LOAD 6
  #define SIM_WALK_UNSUCCESSFUL_STORE 7
  #define SIM_WALK_UNSUCCESSFUL_UNCLASSIFY 8
  #define SIM_WALK_UNSUCCESSFUL_BAD_PADDR_PTR 9
  int translate_virtual_address_api(int procid, const uint64_t* vaddr, int intent, uint64_t* paddr, uint64_t* memattrs);

private:
  isa_parser_t isa;
  const cfg_t * const cfg;
  std::vector<processor_t*> procs;
  std::map<size_t, processor_t*> harts;

  std::vector<std::pair<reg_t, abstract_mem_t*>> mems;
  std::vector<std::pair<reg_t, abstract_mem_t*>> auto_init_mems;
  std::pair<reg_t, reg_t> initrd_range;
  std::string dts;
  std::string dtb;
  bool dtb_enabled;
  std::vector<std::shared_ptr<abstract_device_t>> devices;
  std::shared_ptr<clint_t> clint;
  std::shared_ptr<plic_t> plic;
  bus_t bus;

  log_file_t log_file; // commit log output file
  FILE *cmd_file; // pointer to debug command input file

  // socketif_t *socketif;
  std::ostream sout_; // used for socket and terminal interface

  // size_t current_step;
  // size_t current_proc;
  bool debug;
  bool histogram_enabled; // provide a histogram of PCs
  bool log;

  // memory-mapped I/O routines
  virtual char* addr_to_mem(reg_t paddr) override;
  virtual bool mmio_load(reg_t paddr, size_t len, uint8_t* bytes) override;
  virtual bool mmio_store(reg_t paddr, size_t len, const uint8_t* bytes) override;

  virtual const char* get_symbol(uint64_t paddr) override;

  // sparse memory routines
  Force::Memory _ForceSparseMemoryModel;
  bool is_initialized(reg_t paddr, size_t len);
  void do_initialize(reg_t paddr, size_t numBytes);
  bool is_reserved(reg_t paddr, size_t numBytes);
  void set_reserve(reg_t paddr, size_t numBytes);
  void set_unreserve(reg_t paddr, size_t numBytes);


  // void initialize(reg_t paddr, reg_t value, size_t numBytes, Force::EMemDataType type);
  // void initialize(reg_t paddr, uint8_t *data, uint8_t *attrs, size_t numBytes, Force::EMemDataType type);

  reg_t entry;
  std::map<std::string, uint64_t> load_elf(const char* fn, reg_t* entry);

  reg_t get_mem(const std::vector<std::string>& args);

  friend class processor_t;
  friend class mmu_t;

  // void reset();
  // void clear_chunk(reg_t taddr, size_t len);
  // void read_chunk_partially_initialized(reg_t taddr, size_t len, void* dst);
  // void write_chunk(reg_t taddr, size_t len, const void* src);
  // size_t chunk_align() { return 8; }
  // size_t chunk_max_size() { return 8; }
  // endianness_t get_target_endianness() const;

  // htif
  virtual void reset() override;
  virtual void read_chunk(addr_t taddr, size_t len, void* dst) override;
  virtual void write_chunk(addr_t taddr, size_t len, const void* src) override;
  virtual size_t chunk_align() override { return 8; }
  virtual size_t chunk_max_size() override { return 8; }
  virtual endianness_t get_target_endianness() const override;
  void read_chunk_partially_initialized(reg_t taddr, size_t len, void* dst);
};

#endif

extern volatile bool ctrlc_pressed;

#endif
