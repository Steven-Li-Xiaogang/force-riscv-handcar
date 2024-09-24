// See LICENSE for license details.
#ifndef FORCE_RISCV_ENABLE
#include "config.h"
#include "sim.h"
#include "mmu.h"
#include "dts.h"
#include "remote_bitbang.h"
#include "byteorder.h"
#include "platform.h"
#include "libfdt.h"
#include "socketif.h"
#include <fstream>
#include <map>
#include <iostream>
#include <sstream>
#include <climits>
#include <cstdlib>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

#else
#include "config.h"
#include "sim.h"
#include "mmu.h"
#include "byteorder.h"
#include "platform.h"
#include <fstream>
#include <map>
#include <iostream>
#include <sstream>
#include <climits>
#include <cstdlib>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

#include <fcntl.h>  // for elf loading
#include <sys/mman.h>  // for elf loading
#include <sys/stat.h>  // for elf loading

#include "disasm.h" // for disassembly
#include "elf.h" // for elf loading
#include "memtracer.h" // for address translation

extern "C" {
// update_generator_register function: for the given cpuid, this callback function is called by the simulator to notify the user that a register has been accessed.
//
//  inputs:
//      uint32_t cpuid -- refers to the processor ID
//      const char* registerName -- the name of the reigster (programmer's name)
//      uint64_t value -- the data stored in the register after update
//      uint64_t mask -- 1's indicate relevant bits
//      const char* accessType -- indicates if the access was a read or write.
//
void update_generator_register(uint32_t cpuid, const char* registerName, uint64_t value, uint64_t mask, const char* accessType);  //!< update generator register information when step an instruction
}

#endif

#include "log_print.h"

volatile bool ctrlc_pressed = false;
static void handle_signal(int sig)
{
  if (ctrlc_pressed)
    exit(-1);
  ctrlc_pressed = true;
  signal(sig, &handle_signal);
}

#ifndef FORCE_RISCV_ENABLE

extern device_factory_t* clint_factory;
extern device_factory_t* plic_factory;
extern device_factory_t* ns16550_factory;

sim_t::sim_t(const cfg_t *cfg, bool halted,
             std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
             std::vector<device_factory_t*> plugin_device_factories,
             const std::vector<std::string>& args,
             const debug_module_config_t &dm_config,
             const char *commit_log_file,
             bool dtb_enabled, const char *dtb_file,
             bool socket_enabled,
             FILE *cmd_file) // needed for command line option --cmd
  : htif_t(args),
    isa(cfg->isa, cfg->priv),
    cfg(cfg),
    mems(mems),
    procs(std::max(cfg->nprocs(), size_t(1))),
    dtb_enabled(dtb_enabled),
    log_file(commit_log_file),
    cmd_file(cmd_file),
    sout_(nullptr),
    current_step(0),
    current_proc(0),
    debug(false),
    histogram_enabled(false),
    log(false),
    remote_bitbang(NULL),
    debug_module(this, dm_config)
{
  signal(SIGINT, &handle_signal);

  sout_.rdbuf(std::cerr.rdbuf()); // debug output goes to stderr by default

  for (auto& x : mems)
    bus.add_device(x.first, x.second);

  bus.add_device(DEBUG_START, &debug_module);

  socketif = NULL;
#ifdef HAVE_BOOST_ASIO
  if (socket_enabled) {
    socketif = new socketif_t();
  }
#else
  if (socket_enabled) {
    fputs("Socket support requires compilation with boost asio; "
          "please rebuild the riscv-isa-sim project using "
          "\"configure --with-boost-asio\".\n",
          stderr);
    abort();
  }
#endif

#ifndef RISCV_ENABLE_DUAL_ENDIAN
  if (cfg->endianness != endianness_little) {
    fputs("Big-endian support has not been prroperly enabled; "
          "please rebuild the riscv-isa-sim project using "
          "\"configure --enable-dual-endian\".\n",
          stderr);
    abort();
  }
#endif

  debug_mmu = new mmu_t(this, cfg->endianness, NULL);

  for (size_t i = 0; i < cfg->nprocs(); i++) {
    procs[i] = new processor_t(&isa, cfg, this, cfg->hartids[i], halted,
                               log_file.get(), sout_);
    harts[cfg->hartids[i]] = procs[i];
  }

  // When running without using a dtb, skip the fdt-based configuration steps
  if (!dtb_enabled) return;

  // Only make a CLINT (Core-Local INTerrupt controller) and PLIC (Platform-
  // Level-Interrupt-Controller) if they are specified in the device tree
  // configuration.
  //
  // This isn't *quite* as general as we could get (because you might have one
  // that's not bus-accessible), but it should handle the normal use cases. In
  // particular, the default device tree configuration that you get without
  // setting the dtb_file argument has one.
  std::vector<const device_factory_t*> device_factories = {
    clint_factory, // clint must be element 0
    plic_factory, // plic must be element 1
    ns16550_factory};
  device_factories.insert(device_factories.end(),
                          plugin_device_factories.begin(),
                          plugin_device_factories.end());

  // Load dtb_file if provided, otherwise self-generate a dts/dtb
  if (dtb_file) {
    std::ifstream fin(dtb_file, std::ios::binary);
    if (!fin.good()) {
      std::cerr << "can't find dtb file: " << dtb_file << std::endl;
      exit(-1);
    }
    std::stringstream strstream;
    strstream << fin.rdbuf();
    dtb = strstream.str();
  } else {
    std::pair<reg_t, reg_t> initrd_bounds = cfg->initrd_bounds;
    std::string device_nodes;
    for (const device_factory_t *factory : device_factories)
      device_nodes.append(factory->generate_dts(this));
    dts = make_dts(INSNS_PER_RTC_TICK, CPU_HZ,
                   initrd_bounds.first, initrd_bounds.second,
                   cfg->bootargs, cfg->pmpregions, cfg->pmpgranularity,
                   procs, mems, device_nodes);
    dtb = dts_compile(dts);
  }

  int fdt_code = fdt_check_header(dtb.c_str());
  if (fdt_code) {
    std::cerr << "Failed to read DTB from ";
    if (!dtb_file) {
      std::cerr << "auto-generated DTS string";
    } else {
      std::cerr << "`" << dtb_file << "'";
    }
    std::cerr << ": " << fdt_strerror(fdt_code) << ".\n";
    exit(-1);
  }

  void *fdt = (void *)dtb.c_str();

  for (size_t i = 0; i < device_factories.size(); i++) {
    const device_factory_t *factory = device_factories[i];
    reg_t device_base = 0;
    abstract_device_t* device = factory->parse_from_fdt(fdt, this, &device_base);
    if (device) {
      assert(device_base);
      std::shared_ptr<abstract_device_t> dev_ptr(device);
      add_device(device_base, dev_ptr);

      if (i == 0) // clint_factory
        clint = std::static_pointer_cast<clint_t>(dev_ptr);
      else if (i == 1) // plic_factory
        plic = std::static_pointer_cast<plic_t>(dev_ptr);
    }
  }

  //per core attribute
  int cpu_offset = 0, cpu_map_offset, rc;
  size_t cpu_idx = 0;
  cpu_offset = fdt_get_offset(fdt, "/cpus");
  cpu_map_offset = fdt_get_offset(fdt, "/cpus/cpu-map");
  if (cpu_offset < 0)
    return;

  for (cpu_offset = fdt_get_first_subnode(fdt, cpu_offset); cpu_offset >= 0;
       cpu_offset = fdt_get_next_subnode(fdt, cpu_offset)) {

    if (!(cpu_map_offset < 0) && cpu_offset == cpu_map_offset)
      continue;

    if (cpu_idx >= nprocs())
      break;

    //handle pmp
    reg_t pmp_num, pmp_granularity;
    if (fdt_parse_pmp_num(fdt, cpu_offset, &pmp_num) != 0)
      pmp_num = 0;
    procs[cpu_idx]->set_pmp_num(pmp_num);

    if (fdt_parse_pmp_alignment(fdt, cpu_offset, &pmp_granularity) == 0) {
      procs[cpu_idx]->set_pmp_granularity(pmp_granularity);
    }

    //handle mmu-type
    const char *mmu_type;
    rc = fdt_parse_mmu_type(fdt, cpu_offset, &mmu_type);
    if (rc == 0) {
      procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SBARE);
      if (strncmp(mmu_type, "riscv,sv32", strlen("riscv,sv32")) == 0) {
        procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV32);
      } else if (strncmp(mmu_type, "riscv,sv39", strlen("riscv,sv39")) == 0) {
        procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV39);
      } else if (strncmp(mmu_type, "riscv,sv48", strlen("riscv,sv48")) == 0) {
        procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV48);
      } else if (strncmp(mmu_type, "riscv,sv57", strlen("riscv,sv57")) == 0) {
        procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV57);
      } else if (strncmp(mmu_type, "riscv,sbare", strlen("riscv,sbare")) == 0) {
        //has been set in the beginning
      } else {
        std::cerr << "core ("
                  << cpu_idx
                  << ") has an invalid 'mmu-type': "
                  << mmu_type << ").\n";
        exit(1);
      }
    } else {
      procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SBARE);
    }

    cpu_idx++;
  }

  if (cpu_idx != nprocs()) {
      std::cerr << "core number in dts ("
                <<  cpu_idx
                << ") doesn't match it in command line ("
                << nprocs() << ").\n";
      exit(1);
  }
}

sim_t::~sim_t()
{
  for (size_t i = 0; i < procs.size(); i++)
    delete procs[i];
  delete debug_mmu;
}

int sim_t::run()
{
  if (!debug && log)
    set_procs_debug(true);

  htif_t::set_expected_xlen(isa.get_max_xlen());

  // htif_t::run() will repeatedly call back into sim_t::idle(), each
  // invocation of which will advance target time
  return htif_t::run();
}

void sim_t::step(size_t n)
{
  for (size_t i = 0, steps = 0; i < n; i += steps)
  {
    steps = std::min(n - i, INTERLEAVE - current_step);
    procs[current_proc]->step(steps);

    current_step += steps;
    if (current_step == INTERLEAVE)
    {
      current_step = 0;
      procs[current_proc]->get_mmu()->yield_load_reservation();
      if (++current_proc == procs.size()) {
        current_proc = 0;
        reg_t rtc_ticks = INTERLEAVE / INSNS_PER_RTC_TICK;
        for (auto &dev : devices) dev->tick(rtc_ticks);
      }
    }
  }
}

void sim_t::add_device(reg_t addr, std::shared_ptr<abstract_device_t> dev) {
  bus.add_device(addr, dev.get());
  devices.push_back(dev);
}

void sim_t::set_debug(bool value)
{
  debug = value;
}

void sim_t::set_histogram(bool value)
{
  histogram_enabled = value;
  for (size_t i = 0; i < procs.size(); i++) {
    procs[i]->set_histogram(histogram_enabled);
  }
}

void sim_t::configure_log(bool enable_log, bool enable_commitlog)
{
  log = enable_log;

  if (!enable_commitlog)
    return;

  for (processor_t *proc : procs) {
    proc->enable_log_commits();
  }
}

void sim_t::set_procs_debug(bool value)
{
  for (size_t i=0; i< procs.size(); i++)
    procs[i]->set_debug(value);
}

static bool paddr_ok(reg_t addr)
{
  return (addr >> MAX_PADDR_BITS) == 0;
}

bool sim_t::mmio_load(reg_t paddr, size_t len, uint8_t* bytes)
{
  if (paddr + len < paddr || !paddr_ok(paddr + len - 1))
    return false;
  return bus.load(paddr, len, bytes);
}

bool sim_t::mmio_store(reg_t paddr, size_t len, const uint8_t* bytes)
{
  if (paddr + len < paddr || !paddr_ok(paddr + len - 1))
    return false;
  return bus.store(paddr, len, bytes);
}

void sim_t::set_rom()
{
  const int reset_vec_size = 8;

  reg_t start_pc = cfg->start_pc.value_or(get_entry_point());

  uint32_t reset_vec[reset_vec_size] = {
    0x297,                                      // auipc  t0,0x0
    0x28593 + (reset_vec_size * 4 << 20),       // addi   a1, t0, &dtb
    0xf1402573,                                 // csrr   a0, mhartid
    get_core(0)->get_xlen() == 32 ?
      0x0182a283u :                             // lw     t0,24(t0)
      0x0182b283u,                              // ld     t0,24(t0)
    0x28067,                                    // jr     t0
    0,
    (uint32_t) (start_pc & 0xffffffff),
    (uint32_t) (start_pc >> 32)
  };
  if (get_target_endianness() == endianness_big) {
    int i;
    // Instuctions are little endian
    for (i = 0; reset_vec[i] != 0; i++)
      reset_vec[i] = to_le(reset_vec[i]);
    // Data is big endian
    for (; i < reset_vec_size; i++)
      reset_vec[i] = to_be(reset_vec[i]);

    // Correct the high/low order of 64-bit start PC
    if (get_core(0)->get_xlen() != 32)
      std::swap(reset_vec[reset_vec_size-2], reset_vec[reset_vec_size-1]);
  } else {
    for (int i = 0; i < reset_vec_size; i++)
      reset_vec[i] = to_le(reset_vec[i]);
  }

  std::vector<char> rom((char*)reset_vec, (char*)reset_vec + sizeof(reset_vec));

  rom.insert(rom.end(), dtb.begin(), dtb.end());
  const int align = 0x1000;
  rom.resize((rom.size() + align - 1) / align * align);

  std::shared_ptr<rom_device_t> boot_rom(new rom_device_t(rom));
  add_device(DEFAULT_RSTVEC, boot_rom);
}

char* sim_t::addr_to_mem(reg_t paddr) {
  if (!paddr_ok(paddr))
    return NULL;
  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second))
    if (paddr - desc.first < mem->size())
      return mem->contents(paddr - desc.first);
  return NULL;
}

const char* sim_t::get_symbol(uint64_t paddr)
{
  return htif_t::get_symbol(paddr);
}

// htif

void sim_t::reset()
{
  if (dtb_enabled)
    set_rom();
}

void sim_t::idle()
{
  if (done())
    return;

  if (debug || ctrlc_pressed)
    interactive();
  else
    step(INTERLEAVE);

  if (remote_bitbang)
    remote_bitbang->tick();
}

void sim_t::read_chunk(addr_t taddr, size_t len, void* dst)
{
  assert(len == 8);
  auto data = debug_mmu->to_target(debug_mmu->load<uint64_t>(taddr));
  memcpy(dst, &data, sizeof data);
}

void sim_t::write_chunk(addr_t taddr, size_t len, const void* src)
{
  assert(len == 8);
  target_endian<uint64_t> data;
  memcpy(&data, src, sizeof data);
  debug_mmu->store<uint64_t>(taddr, debug_mmu->from_target(data));
}

endianness_t sim_t::get_target_endianness() const
{
  return debug_mmu->is_target_big_endian()? endianness_big : endianness_little;
}

void sim_t::proc_reset(unsigned id)
{
  debug_module.proc_reset(id);
}

#else
simlib_t::simlib_t(const cfg_t *cfg, bool halted, bool auto_init_mem,
                   std::vector<std::pair<reg_t, abstract_mem_t*>> mems,
                   std::vector<device_factory_t*> plugin_device_factories,
                   const std::vector<std::string>& args,
                   const char *commit_log_file, // instruction commit log
                   FILE *cmd_file) // needed for command line option --cmd
  : htif_t(args),
    isa(cfg->isa, cfg->priv),
    cfg(cfg),
    procs(std::max(cfg->nprocs(), size_t(1))),
    log_file(commit_log_file),
    cmd_file(cmd_file),
    sout_(nullptr),
    // current_step(0),
    // current_proc(0),
    debug(false),
    histogram_enabled(false),
    _ForceSparseMemoryModel(Force::EMemBankType::Default, auto_init_mem),
    entry(DRAM_BASE)
{
  signal(SIGINT, &handle_signal);

  sout_.rdbuf(std::cerr.rdbuf()); // debug output goes to stderr by default


#ifndef RISCV_ENABLE_DUAL_ENDIAN
  if (cfg->endianness != endianness_little) {
    fputs("Big-endian support has not been prroperly enabled; "
          "please rebuild the riscv-isa-sim project using "
          "\"configure --enable-dual-endian\".\n",
          stderr);
    abort();
  }
#endif

  debug_mmu = new mmu_t(this, cfg->endianness, NULL);

  assert(cfg->nprocs() != 0);

  for (size_t i = 0; i < cfg->nprocs(); i++) {
    procs[i] = new processor_t(&isa, cfg, this, cfg->hartids[i], halted,
                               log_file.get(), sout_);
    harts[cfg->hartids[i]] = procs[i];
  }

  // //per core attribute
  // int cpu_offset = 0, cpu_map_offset, rc;
  // size_t cpu_idx = 0;
  // cpu_offset = fdt_get_offset(fdt, "/cpus");
  // cpu_map_offset = fdt_get_offset(fdt, "/cpus/cpu-map");
  // if (cpu_offset < 0)
  //   return;

  // for (cpu_offset = fdt_get_first_subnode(fdt, cpu_offset); cpu_offset >= 0;
  //      cpu_offset = fdt_get_next_subnode(fdt, cpu_offset)) {

  //   if (!(cpu_map_offset < 0) && cpu_offset == cpu_map_offset)
  //     continue;

  //   if (cpu_idx >= nprocs())
  //     break;

  //   //handle pmp
  //   reg_t pmp_num, pmp_granularity;
  //   if (fdt_parse_pmp_num(fdt, cpu_offset, &pmp_num) != 0)
  //     pmp_num = 0;
  //   procs[cpu_idx]->set_pmp_num(pmp_num);

  //   if (fdt_parse_pmp_alignment(fdt, cpu_offset, &pmp_granularity) == 0) {
  //     procs[cpu_idx]->set_pmp_granularity(pmp_granularity);
  //   }

  //   //handle mmu-type
  //   const char *mmu_type;
  //   rc = fdt_parse_mmu_type(fdt, cpu_offset, &mmu_type);
  //   if (rc == 0) {
  //     procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SBARE);
  //     if (strncmp(mmu_type, "riscv,sv32", strlen("riscv,sv32")) == 0) {
  //       procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV32);
  //     } else if (strncmp(mmu_type, "riscv,sv39", strlen("riscv,sv39")) == 0) {
  //       procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV39);
  //     } else if (strncmp(mmu_type, "riscv,sv48", strlen("riscv,sv48")) == 0) {
  //       procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV48);
  //     } else if (strncmp(mmu_type, "riscv,sv57", strlen("riscv,sv57")) == 0) {
  //       procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV57);
  //     } else if (strncmp(mmu_type, "riscv,sbare", strlen("riscv,sbare")) == 0) {
  //       //has been set in the beginning
  //     } else {
  //       std::cerr << "core ("
  //                 << cpu_idx
  //                 << ") has an invalid 'mmu-type': "
  //                 << mmu_type << ").\n";
  //       exit(1);
  //     }
  //   } else {
  //     procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SBARE);
  //   }

  //   cpu_idx++;
  // }

  // if (cpu_idx != nprocs()) {
  //     std::cerr << "core number in dts ("
  //               <<  cpu_idx
  //               << ") doesn't match it in command line ("
  //               << nprocs() << ").\n";
  //     exit(1);
  // }
}

simlib_t::~simlib_t()
{
  for (size_t i = 0; i < procs.size(); i++)
    delete procs[i];

  for (auto mem : auto_init_mems)
    delete mem.second;

  delete debug_mmu;
}

int simlib_t::step_simulator(int target_id, int num_steps, int stx_failed)
{
  LOG_PRINT_ERROR("[simlib_t::%s] target id=%d, num_steps=%d\n", __func__, target_id, num_steps);

  processor_t* proc = get_core(target_id);
  if (proc == nullptr)
    return 1;

  proc->step(num_steps);
  const char pc_name[] = "PC\0";
  const uint64_t pc_mask = 0xFFFFFFFFFFFFFFFF;
  uint64_t pc_value = 0x0;
  const char pc_update_access_type[] = "write\0";
  get_pc_api(target_id, pc_name, reinterpret_cast<uint8_t*>(&pc_value), sizeof(uint64_t));
  update_generator_register(target_id, pc_name, pc_value, pc_mask, pc_update_access_type);

  return 0;
}

void simlib_t::set_debug(bool value)
{
  debug = value;
}

int simlib_t::get_disassembly(int target_id, const uint64_t* pc, char** opcode, char** disassembly)
{
  std::string opcode_temp;
  std::string disassembly_temp;
  uint64_t opcode_num = 0ull;

  if(procs.size() > 0)
  {
    processor_t* proc = get_core(target_id);
    if (proc == nullptr)
      return 1;
    try {
      const disassembler_t* disassembler = proc->get_disassembler(); // should be OK to get processor's disassembler, using it should be idempotent

      // currently this fails a check in the Spike code if the pc isn't found so the code does not have the opportunity to return 1.
      insn_fetch_t fetched = proc->get_mmu()->load_insn(*pc);
      opcode_num = fetched.insn.bits() & ((1ull << (8 * insn_length(fetched.insn.bits()))) - 1); //This is the conversion the processor_t::disasm(...) uses to format the opcode for output.
      disassembly_temp = disassembler->disassemble(fetched.insn);
      // format the string interpretation of the opcode as a hex number.
      std::stringstream stream;
      stream << "0x" << std::hex << opcode_num;
      opcode_temp = stream.str();
    } catch(...) {
      strcpy(*opcode,"00000000");
      strcpy(*disassembly,"?");
      return 1; // there may not be an instruction at the PC (example: page fault on branch)
    }
  } else {
    return 2; // No processors, therefore no configured disassembler for us to use
  }

  // At this point disassembly proceeded correctly. Now check the output buffers
  if(opcode == nullptr || disassembly == nullptr) {
    return 2;
  }
  size_t opcode_buffer_length = opcode != nullptr ? strlen(*opcode) : 0;
  size_t disassembly_buffer_length = disassembly != nullptr ? strlen(*disassembly) : 0;
  // check for enough size to place the null termination character
  if((opcode_buffer_length < (opcode_temp.size()+1)) || (disassembly_buffer_length < (disassembly_temp.size()+1))) {
    return 2; // The user didn't give us room to put the answers
  }

  // Warning: the 'insn't' null and string length tests above can fail to uncover a corner case of bad input, where a pointer to a string literal is aliased by a char* pointer.
  // The compiler will accept this, but it will fail when you try to copy as in below, basically be sure your string buffer was correctly allocated.
  opcode_temp.copy(*opcode, opcode_temp.size(), 0);
  (*opcode)[opcode_temp.size()]='\0'; // No idea how the user is initializing these strings, so just null terminate the relevant part.
  disassembly_temp.copy(*disassembly, disassembly_temp.size(), 0);
  (*disassembly)[disassembly_temp.size()]='\0';

  return 0;
}

void simlib_t::set_log(bool enable_log, bool enable_commit_log)
{
  log = enable_log;

  if (enable_commit_log) {
    for (processor_t *proc : procs)
      proc->enable_log_commits();
  }
}

void simlib_t::set_histogram(bool histogram)
{
  histogram_enabled = histogram;
  for (size_t i = 0; i < procs.size(); i++) {
    procs[i]->set_histogram(histogram_enabled);
  }
}

void simlib_t::set_procs_debug(bool proc_debug)
{
  for (size_t i=0; i< procs.size(); i++)
    procs[i]->set_debug(proc_debug);
}

std::map<std::string, uint64_t> simlib_t::load_elf(const char* fn, reg_t* entry)
{
  int fd = open(fn, O_RDONLY);
  struct stat s;
  assert(fd != -1);
  if (fstat(fd, &s) < 0)
    abort();
  size_t size = s.st_size;

  char* buf = (char*)mmap(NULL, size, PROT_READ, MAP_PRIVATE, fd, 0);
  assert(buf != MAP_FAILED);
  close(fd);

  assert(size >= sizeof(Elf64_Ehdr));
  const Elf64_Ehdr* eh64 = (const Elf64_Ehdr*)buf;
  assert(IS_ELF32(*eh64) || IS_ELF64(*eh64));

  std::vector<uint8_t> zeros;
  std::vector<uint8_t> attrs;
  std::map<std::string, uint64_t> symbols;

  #define LOAD_ELF(ehdr_t, phdr_t, shdr_t, sym_t) do { \
    ehdr_t* eh = (ehdr_t*)buf; \
    phdr_t* ph = (phdr_t*)(buf + eh->e_phoff); \
    *entry = eh->e_entry; \
    assert(size >= eh->e_phoff + eh->e_phnum*sizeof(*ph)); \
    for (unsigned i = 0; i < eh->e_phnum; i++) { \
      if(ph[i].p_type == PT_LOAD && ph[i].p_memsz) { \
        if (ph[i].p_filesz) { \
          assert(size >= ph[i].p_offset + ph[i].p_filesz); \
          attrs.resize(ph[i].p_filesz); \
          std::fill(attrs.begin(), attrs.end(), 0); \
          initialize_multiword(ph[i].p_paddr, ph[i].p_filesz, (uint8_t*)buf + ph[i].p_offset); \
        } \
        zeros.resize(ph[i].p_memsz - ph[i].p_filesz); \
        initialize_multiword(ph[i].p_paddr + ph[i].p_filesz, ph[i].p_memsz - ph[i].p_filesz, &zeros[0]); \
      } \
    } \
    shdr_t* sh = (shdr_t*)(buf + eh->e_shoff); \
    assert(size >= eh->e_shoff + eh->e_shnum*sizeof(*sh)); \
    assert(eh->e_shstrndx < eh->e_shnum); \
    assert(size >= sh[eh->e_shstrndx].sh_offset + sh[eh->e_shstrndx].sh_size); \
    char *shstrtab = buf + sh[eh->e_shstrndx].sh_offset; \
    unsigned strtabidx = 0, symtabidx = 0; \
    for (unsigned i = 0; i < eh->e_shnum; i++) { \
      unsigned max_len = sh[eh->e_shstrndx].sh_size - sh[i].sh_name; \
      assert(sh[i].sh_name < sh[eh->e_shstrndx].sh_size); \
      assert(strnlen(shstrtab + sh[i].sh_name, max_len) < max_len); \
      if (sh[i].sh_type & SHT_NOBITS) continue; \
      assert(size >= sh[i].sh_offset + sh[i].sh_size); \
      if (strcmp(shstrtab + sh[i].sh_name, ".strtab") == 0) \
        strtabidx = i; \
      if (strcmp(shstrtab + sh[i].sh_name, ".symtab") == 0) \
        symtabidx = i; \
    } \
    if (strtabidx && symtabidx) { \
      char* strtab = buf + sh[strtabidx].sh_offset; \
      sym_t* sym = (sym_t*)(buf + sh[symtabidx].sh_offset); \
      for (unsigned i = 0; i < sh[symtabidx].sh_size/sizeof(sym_t); i++) { \
        unsigned max_len = sh[strtabidx].sh_size - sym[i].st_name; \
        assert(sym[i].st_name < sh[strtabidx].sh_size); \
        assert(strnlen(strtab + sym[i].st_name, max_len) < max_len); \
        symbols[strtab + sym[i].st_name] = sym[i].st_value; \
      } \
    } \
  } while(0)

  if (IS_ELF32(*eh64))
    LOAD_ELF(Elf32_Ehdr, Elf32_Phdr, Elf32_Shdr, Elf32_Sym);
  else
    LOAD_ELF(Elf64_Ehdr, Elf64_Phdr, Elf64_Shdr, Elf64_Sym);

  munmap(buf, size);
  return symbols;
}

// Not only loads the elf file in the specified path into the memory model but readies the simulator for stepping.
int simlib_t::load_program_now(const char* elfPath)
{
  std::string path;
  if (access(elfPath, F_OK) == 0)
    path = elfPath;

  if (path.empty()) {
    std::cerr << "could not open " << elfPath << " (did you misspell it? If VCS, did you forget +permissive/+permissive-off?)" << std::endl;
    return 1;
  }

  std::map<std::string, uint64_t> symbols = load_elf(path.c_str(), &entry);
  std::cerr << "============" << std::hex << entry << "=================" << std::endl;

  reset();

  return 0;
}

void simlib_t::reset()
{
  reg_t start_pc = cfg->start_pc.value_or(get_entry_point());

  for (size_t i = 0; i < procs.size(); i++)
    procs[i]->reset_pc_api(entry);

  std::cerr << "<<<<<<<<<<<<<< simlib_t::reset (pc=" << std::hex << start_pc << ") >>>>>>>>>>>>>>>" << std::endl;
}

static bool paddr_ok(reg_t addr)
{
  return (addr >> MAX_PADDR_BITS) == 0;
}

// bool simlib_t::mmio_load(reg_t paddr, size_t len, uint8_t* bytes)
// {
//     return false;
// }

// bool simlib_t::mmio_store(reg_t paddr, size_t len, const uint8_t* bytes)
// {
//     return false;
// }

bool simlib_t::mmio_load(reg_t paddr, size_t len, uint8_t* bytes)
{
  if (paddr + len < paddr || !paddr_ok(paddr + len - 1))
    return false;
  return bus.load(paddr, len, bytes);
}

bool simlib_t::mmio_store(reg_t paddr, size_t len, const uint8_t* bytes)
{
  if (paddr + len < paddr || !paddr_ok(paddr + len - 1))
    return false;
  return bus.store(paddr, len, bytes);
}

char* simlib_t::addr_to_mem(reg_t paddr) {
  if (!paddr_ok(paddr))
    return NULL;
  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second))
    if (paddr - desc.first < mem->size())
      return mem->contents(paddr - desc.first);
  return NULL;
}

#ifdef FORCE_RISCV_ENABLE
bool simlib_t::is_initialized(reg_t paddr, size_t len) {
  if (!paddr_ok(paddr))
    return false;

  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second))
    if (paddr - desc.first < mem->size())
      return mem->contents_initialized(paddr - desc.first, len);
  return false;
}

void simlib_t::do_initialize(reg_t paddr, size_t numBytes) {
  if (!paddr_ok(paddr))
    return ;

  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second)) { // existed mem node
    if (paddr - desc.first < mem->size())
      mem->set_initialized(paddr - desc.first, numBytes);
  } else {
    reg_t paddr_aligned = paddr & PGMASK;
    reg_t size_aligned = numBytes;
    if (paddr & (PGSIZE-1)) // not aligned
      size_aligned += (paddr & (PGSIZE-1));
    if ((paddr + numBytes) & (PGSIZE-1))
      size_aligned += PGSIZE - ((paddr + numBytes) & (PGSIZE-1));
    std::pair<reg_t, abstract_mem_t*> memnode = std::make_pair(paddr_aligned, new mem_t(size_aligned));
    memnode.second->set_initialized(paddr - paddr_aligned, numBytes);

    auto_init_mems.push_back(memnode);
    bus.add_device(memnode.first, memnode.second);
  }
}

bool simlib_t::is_reserved(reg_t paddr, size_t numBytes) {
  if (!paddr_ok(paddr))
    return false;
  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second))
    if (paddr - desc.first < mem->size() && paddr - desc.first + numBytes < mem->size())
      return mem->contents_reserved();
  return false;
}

void simlib_t::set_reserve(reg_t paddr, size_t numBytes) {
  if (!paddr_ok(paddr))
    return ;

  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second)) { // reserve an existed mem node
    if (paddr - desc.first < mem->size())
      mem->set_reserved(true);
  } else { // reserve by creating new mem node
    reg_t paddr_aligned = paddr & PGMASK;
    reg_t size_aligned = numBytes;
    if (paddr & (PGSIZE-1)) // not aligned
      size_aligned += (paddr & (PGSIZE-1));
    if ((paddr + numBytes) & (PGSIZE-1))
      size_aligned += PGSIZE - ((paddr + numBytes) & (PGSIZE-1));
    std::pair<reg_t, abstract_mem_t*> memnode = std::make_pair(paddr_aligned, new mem_t(size_aligned));
    memnode.second->set_reserved(true);

    auto_init_mems.push_back(memnode);
    bus.add_device(memnode.first, memnode.second);
  }
}

void simlib_t::set_unreserve(reg_t paddr, size_t numBytes) {
  if (paddr_ok(paddr))
    return ;

  auto desc = bus.find_device(paddr);
  if (auto mem = dynamic_cast<abstract_mem_t*>(desc.second)) { // unreserve an existed mem node
    mem->set_reserved(false);
  }
}
#endif

const char* simlib_t::get_symbol(uint64_t paddr)
{
  return htif_t::get_symbol(paddr);
}

#include <fmt.h>

// #define FORCE_RISCV_SPIKE_MEM

uint64_t simlib_t::sparse_read(reg_t paddr, size_t len)
{
  LOG_PRINT_NOTICE("[simlib_t::%s] paddr=%lx, len=%ld\n", __func__, paddr, len);

#ifndef FORCE_RISCV_SPIKE_MEM
  uint64_t rdata = _ForceSparseMemoryModel.Read(paddr, len);
  LOG_PRINT_NOTICE("[simlib_t::%s] done %lx\n", __func__, rdata);
  return rdata;
#else
  uint64_t rdata = 0;
  uint8_t* pdata = (uint8_t*)&rdata;
  auto host_addr = addr_to_mem(paddr);
  assert(len <= 8);

  // to value from be
  for (size_t i = 0; i < len; i++)
    pdata[i] = host_addr[len-i-1];
  LOG_PRINT_NOTICE("[simlib_t::%s] done %lx\n", __func__, rdata);
  return rdata;
#endif
}

void simlib_t::sparse_read_partially_initialized(reg_t paddr, size_t len, uint8_t* bytes)
{
  LOG_PRINT_NOTICE("[simlib_t::%s] paddr=%lx, len=%ld\n", __func__, paddr, len);

#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.ReadPartiallyInitialized(paddr, len, bytes);
#else
  auto host_addr = addr_to_mem(paddr);
  for (reg_t addr = paddr; addr < paddr + len; addr++) {
    if (is_initialized(addr, 1))
      bytes[addr - paddr] = host_addr[addr - paddr];
    else
      bytes[addr - paddr] = 0; // default pattern
  }
#endif
}

void simlib_t::sparse_write(reg_t paddr, const uint8_t* bytes, size_t len)
{
  LOG_PRINT_NOTICE("[simlib_t::%s] paddr=%lx, len=%ld\n", __func__, paddr, len);
  LOG_PRINT_MEMORY(bytes, len);

#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Write(paddr, bytes, len);
#else
  auto host_addr = addr_to_mem(paddr);
  memcpy(host_addr, bytes, len);
#endif
}

void simlib_t::sparse_write(reg_t paddr, uint64_t value, size_t len)
{
  LOG_PRINT_NOTICE("[simlib_t::%s] paddr=%lx, value=%lx, len=%ld\n", __func__, paddr, value, len);

#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Write(paddr, value, len);
#else
  auto host_addr = addr_to_mem(paddr);
  memcpy(host_addr, &value, len);
#endif
}

void simlib_t::sparse_write_with_initialization(reg_t paddr, const uint8_t* bytes, size_t len)
{
  LOG_PRINT_NOTICE("[simlib_t::%s] paddr=%lx, len=%ld\n", __func__, paddr, len);
  LOG_PRINT_MEMORY(bytes, len);

#ifndef FORCE_RISCV_SPIKE_MEM
  if (not sparse_is_pa_initialized(paddr, len)) {
    _ForceSparseMemoryModel.AutoInitialize(paddr, len);
  }

  _ForceSparseMemoryModel.Write(paddr, bytes, len);
#else
  if (not sparse_is_pa_initialized(paddr, len))
    sparse_initialize_pa(paddr, bytes, nullptr, len, Force::EMemDataType::Both);

  auto host_addr = addr_to_mem(paddr);
  memcpy(host_addr, bytes, len);
#endif
}

bool simlib_t::sparse_is_pa_initialized(reg_t paddr, size_t len)
{
  bool initialized;
  LOG_PRINT_INFO("[simlib_t::%s] paddr=%lx, len=%ld\n", __func__, paddr, len);

#ifndef FORCE_RISCV_SPIKE_MEM
  initialized =  _ForceSparseMemoryModel.IsInitialized(paddr, len);
#else
  initialized = is_initialized(paddr, len);
#endif
  LOG_PRINT_INFO("[simlib_t::%s] %s\n", __func__, initialized ? "true" : "false");
  return initialized;
}

void simlib_t::sparse_initialize_pa(reg_t paddr, const uint8_t* data, const uint8_t* attrs, uint32_t nBytes, Force::EMemDataType type)
{
  LOG_PRINT_INFO("[simlib_t::%s:%d] paddr=%lx, nBytes=%d\n", __func__, __LINE__, paddr, nBytes);
  LOG_PRINT_MEMORY(data, nBytes, 0);

#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Initialize(paddr, data, attrs, nBytes, type);
#else
  
  auto host_addr = addr_to_mem(paddr);
  if (nullptr == host_addr) {
    reg_t paddr_aligned = paddr & PGMASK;
    reg_t size_aligned = nBytes;
    if (paddr & (PGSIZE-1)) // not aligned
      size_aligned += (paddr & (PGSIZE-1));
    if ((paddr + nBytes) & (PGSIZE-1))
      size_aligned += PGSIZE - ((paddr + nBytes) & (PGSIZE-1));
    std::pair<reg_t, abstract_mem_t*> memnode = std::make_pair(paddr_aligned, new mem_t(size_aligned));
    auto_init_mems.push_back(memnode);
    bus.add_device(memnode.first, memnode.second);
  }

  host_addr = addr_to_mem(paddr);
  if (nullptr == host_addr) {
    LOG_PRINT_ERROR("[simlib_t::%s:%d] can not find host of paddr=%lx\n", __func__, __LINE__, paddr);
    assert(false);
  }

  memcpy(host_addr, data, nBytes);
  do_initialize(paddr, nBytes);
#endif
}

void simlib_t::sparse_initialize_pa(reg_t paddr, reg_t value, size_t numBytes, Force::EMemDataType type)
{
  LOG_PRINT_NOTICE("[simlib_t::%s:%d] paddr=%lx, value=%lx, numBytes=%ld\n", __func__, __LINE__, paddr, value, numBytes);
#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Initialize(paddr, value, numBytes, type);
#else

  auto host_addr = addr_to_mem(paddr);
  if (nullptr == host_addr) { // create memory node while initialization
    reg_t paddr_aligned = paddr & PGMASK;
    reg_t size_aligned = numBytes;
    if (paddr & (PGSIZE-1)) // not aligned
      size_aligned += (paddr & (PGSIZE-1));
    if ((paddr + numBytes) & (PGSIZE-1))
      size_aligned += PGSIZE - ((paddr + numBytes) & (PGSIZE-1));
    std::pair<reg_t, abstract_mem_t*> memnode = std::make_pair(paddr_aligned, new mem_t(size_aligned));
    auto_init_mems.push_back(memnode);
    bus.add_device(memnode.first, memnode.second);
  }

  host_addr = addr_to_mem(paddr);
  if (nullptr == host_addr) {
    LOG_PRINT_INFO("[simlib_t::%s:%d] can not find host of paddr=%lx\n", __func__, __LINE__, paddr);
    assert(false);
  }

  memcpy(host_addr, &value, numBytes);
  do_initialize(paddr, numBytes);
#endif
}

void simlib_t::sparse_initialize_pa(reg_t paddr, reg_t value, size_t numBytes)
{
  LOG_PRINT_NOTICE("[simlib_t::%s:%d] paddr=%lx, value=%lx, numBytes=%ld\n", __func__, __LINE__, paddr, value, numBytes);
#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Initialize(paddr, value, numBytes, Force::EMemDataType::Both);
#else

  auto host_addr = addr_to_mem(paddr);
  if (nullptr == host_addr) { // create memory node while initialization
    reg_t paddr_aligned = paddr & PGMASK;
    reg_t size_aligned = numBytes;
    if (paddr & (PGSIZE-1)) // not aligned
      size_aligned += (paddr & (PGSIZE-1));
    if ((paddr + numBytes) & (PGSIZE-1))
      size_aligned += PGSIZE - ((paddr + numBytes) & (PGSIZE-1));
    std::pair<reg_t, abstract_mem_t*> memnode = std::make_pair(paddr_aligned, new mem_t(size_aligned));
    auto_init_mems.push_back(memnode);
    bus.add_device(memnode.first, memnode.second);
  }

  host_addr = addr_to_mem(paddr);
  if (nullptr == host_addr) {
    LOG_PRINT_INFO("[simlib_t::%s:%d] can not find host of paddr=%lx\n", __func__, __LINE__, paddr);
    assert(false);
  }

  memcpy(host_addr, &value, numBytes);
  do_initialize(paddr, numBytes);
#endif
}

void simlib_t::sparse_reserve(reg_t paddr, size_t numBytes)
{
  LOG_PRINT_NOTICE("[simlib_t::%s:%d] paddr=%lx, numBytes=%ld\n", __func__, __LINE__, paddr, numBytes);
#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Reserve(paddr, numBytes);
#else
  set_reserve(paddr, numBytes);
#endif
}

void simlib_t::sparse_unreserve(reg_t paddr, size_t numBytes)
{
  LOG_PRINT_NOTICE("[simlib_t::%s:%d] paddr=%lx, numBytes=%ld\n", __func__, __LINE__, paddr, numBytes);
#ifndef FORCE_RISCV_SPIKE_MEM
  _ForceSparseMemoryModel.Unreserve(paddr, numBytes);
#else
  set_unreserve(paddr, numBytes);
#endif
}

bool simlib_t::sparse_is_reserved(reg_t paddr, size_t numBytes)
{
#ifndef FORCE_RISCV_SPIKE_MEM
  bool reserved = _ForceSparseMemoryModel.IsReserved(paddr, numBytes);
  LOG_PRINT_NOTICE("[simlib_t::%s:%d] paddr=%lx %s\n", __func__, __LINE__, paddr, reserved ? "true" : "false");
  return reserved;
#else
  LOG_PRINT_NOTICE("[simlib_t::%s:%d] paddr=%lx false\n", __func__, __LINE__, paddr);
  return is_reserved(paddr, numBytes);
#endif
}

// To support multiword initializations during elf loading
void simlib_t::initialize_multiword(reg_t addr, size_t len, const void* bytes)
{
  LOG_PRINT_NOTICE("[simlib_t::%s] addr=%lx, len=%ld\n", __func__, addr, len);
  LOG_PRINT_MEMORY((const uint8_t*)bytes, len);

  size_t align = chunk_align();
  if (len && (addr & (align-1)))
  {
    size_t this_len = std::min(len, align - size_t(addr & (align-1)));
    uint8_t chunk[align];

    read_chunk_partially_initialized(addr & ~(align-1), align, chunk);
    memcpy(chunk + (addr & (align-1)), bytes, this_len);
    write_chunk(addr & ~(align-1), align, chunk);

    bytes = (char*)bytes + this_len;
    addr += this_len;
    len -= this_len;
  }

  if (len & (align-1))
  {
    size_t this_len = len & (align-1);
    size_t start = len - this_len;
    uint8_t chunk[align];

    read_chunk_partially_initialized(addr + start, align, chunk);
    memcpy(chunk, (char*)bytes + start, this_len);
    write_chunk(addr + start, align, chunk);

    len -= this_len;
  }

  // now we're aligned
  bool all_zero = len != 0;
  for (size_t i = 0; i < len; i++)
    all_zero &= ((const char*)bytes)[i] == 0;

  if (all_zero) {
    clear_chunk(addr, len);
  } else {
    size_t max_chunk = chunk_max_size();
    for (size_t pos = 0; pos < len; pos += max_chunk)
      write_chunk(addr + pos, std::min(max_chunk, len - pos), (char*)bytes + pos);
  }
}

// void simlib_t::clear_chunk(reg_t taddr, size_t len)
// {
//   LOG_PRINT_DEBUG("[simlib_t::%s] taddr=%lx, len=%ld\n", __func__, taddr, len);

//   char zeros[chunk_max_size()];
//   memset(zeros, 0, chunk_max_size());

//   for (size_t pos = 0; pos < len; pos += chunk_max_size())
//     write_chunk(taddr + pos, std::min(len - pos, chunk_max_size()), zeros);
// }

void simlib_t::read_chunk_partially_initialized(reg_t taddr, size_t len, void* dst)
{
  LOG_PRINT_DEBUG("[simlib_t::%s] taddr=%lx, len=%ld\n", __func__, taddr, len);

  assert(len == 8);
  auto data = debug_mmu->to_target(debug_mmu->load_partially_initialized<uint64_t>(taddr));
  memcpy(dst, &data, sizeof data);
}

// force riscv new
void simlib_t::read_chunk(addr_t taddr, size_t len, void* dst)
{
  assert(len == 8);
  auto data = debug_mmu->to_target(debug_mmu->load<uint64_t>(taddr));
  memcpy(dst, &data, sizeof data);
}

void simlib_t::write_chunk(reg_t taddr, size_t len, const void* src)
{
  LOG_PRINT_DEBUG("[simlib_t::%s] taddr=%lx, len=%ld\n", __func__, taddr, len);

  assert(len == 8);
  target_endian<uint64_t> data;
  memcpy(&data, src, sizeof data);
  debug_mmu->store<uint64_t>(taddr, debug_mmu->from_target(data));
}

endianness_t simlib_t::get_target_endianness() const
{
  return debug_mmu->is_target_big_endian()? endianness_big : endianness_little;
}

void simlib_t::proc_reset(unsigned id)
{
}

reg_t simlib_t::get_mem(const std::vector<std::string>& args)
{
  std::string addr_str = args[0];
  mmu_t* mmu = debug_mmu;
  if(args.size() == 2)
  {
    unsigned long id = strtoul(addr_str.c_str(), 0, 10);
    processor_t *p = get_core(id);
    mmu = p->get_mmu();
    addr_str = args[1];
  }

  reg_t addr = strtol(addr_str.c_str(),NULL,16), val;
  if(addr == LONG_MAX)
    addr = strtoul(addr_str.c_str(),NULL,16);

  switch(addr % 8)
  {
    case 0:
      val = mmu->load<uint64_t>(addr);
      break;
    case 4:
      val = mmu->load<uint32_t>(addr);
      break;
    case 2:
    case 6:
      val = mmu->load<uint16_t>(addr);
      break;
    default:
      val = mmu->load<uint8_t>(addr);
      break;
  }
  return val;
}

uint64_t simlib_t::get_csr_number(const std::string& input_name)
{
  //Cant use a switch here unless we map the names to ints beforehand
  #define DECLARE_CSR(name, number) if (input_name == #name) return number;
  #include "encoding.h"              // generates if's for all csrs
  return 0xDEADBEEFDEADBEEF;         // else return a value outside the ISA established 4096 possible
  #undef DECLARE_CSR
}

uint64_t simlib_t::get_xpr_number(const std::string& input_name)
{
  return std::find(xpr_arch_name, xpr_arch_name + NXPR, input_name) - xpr_arch_name;
}

uint64_t simlib_t::get_fpr_number(const std::string& input_name)
{
  return std::find(fpr_arch_name, fpr_arch_name + NFPR, input_name) - fpr_arch_name;
}

uint64_t simlib_t::get_vecr_number(const std::string& input_name)
{
  return std::find(vr_name, vr_name + NVPR, input_name) - vr_name;
}

std::string simlib_t::get_csr_name(uint64_t index)
{
  if(index < NCSR)
    return csr_name(index);
  else
    return "unknown-csr";
}

std::string simlib_t::get_xpr_name(uint64_t index)
{
  if(index < NXPR)
    return xpr_arch_name[index];
  else
    return "unknown-xpr";
}

std::string simlib_t::get_fpr_name(uint64_t index)
{
  if(index < NFPR)
    return fpr_arch_name[index];
  else
    return "unknown-fpr";
}

std::string simlib_t::get_vecr_name(uint64_t index)
{
  if(index < NVPR)
    return vr_name[index];
  else
    return "unknown-vr";
}

int simlib_t::read_csr(uint32_t procid, uint64_t index, uint64_t* value, uint32_t* length)
{
  //Check if the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index is valid
  if(index >= NCSR)
    return 3;

  //Check if the index corresponds to a defined csr
  // WARNING: there are a number of CSRs that are defined in encoding.h but have no entry in the processor class get_csr method. In this case, for now, a value of 0xDEADBEEFDEADBEEF will be returned
  std::string temp_name = get_csr_name(index);
  size_t unknown = temp_name.find("unknown");
  if(unknown != std::string::npos)
    return 3;

  //All checks pass, so go ahead and load the value buffer and set the length to indicate the used bytes in the value buffer
  *value = get_core(procid)->get_csr_api(index);
  *length = get_core(procid)->get_xlen() / 8;

  return 0;
}

int simlib_t::read_csr(uint32_t procid, const std::string& input_name, uint64_t* value, uint32_t* length)
{
  //Check if the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the name is valid
  uint64_t index = get_csr_number(input_name);
  if(index >= NCSR)
    return 3;

  //All checks pass, so go ahead and load the value buffer and set the length to indicate the used bytes in the value buffer
  *value = get_core(procid)->get_csr_api(index);
  *length = get_core(procid)->get_xlen() / 8;

  return 0;
}

int simlib_t::read_xpr(uint32_t procid, const std::string& input_name, uint64_t* value, uint32_t* length)
{
  //Check if the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the name is valid
  uint64_t index = get_xpr_number(input_name);
  if(index >= NXPR)
    return 3;

  //All checks pass, so go ahead and load the value buffer and set the length to indicate the used bytes in the value buffer
  *value = get_core(procid)->get_state()->XPR.read_no_callback(index);
  *length = get_core(procid)->get_xlen() / 8;

  return 0;
}

int simlib_t::read_xpr(uint32_t procid, uint64_t index, uint64_t* value, uint32_t* length)
{
  //Check if the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //check if the index provided is valid
  if(index >= NXPR)
    return 3;

  //All checks pass, so go ahead and load the value buffer and set the length to indicate the used bytes in the value buffer
  //*value = get_core(procid)->get_state()->XPR[index];

  *value = get_core(procid)->get_state()->XPR.read_no_callback(index);
  *length = get_core(procid)->get_xlen() / 8;

  return 0;
}

int simlib_t::read_fpr(uint32_t procid, const std::string& input_name, uint8_t* value, uint32_t* length)
{
  //Check if the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //check if the name provided is valid
  uint64_t index = get_fpr_number(input_name);
  if(index >= NFPR)
    return 3;

  //Check that the advertised length of the provided buffer is sufficient
  if(*length < sizeof(freg_t))
    return 4;

  //All checks have passed, so go ahead and load the value buffer and set the length to indicate the used bytes in the value buffer
  freg_t temp_fpr_val = get_core(procid)->get_state()->FPR.read_no_callback(index);
  memcpy(value, &temp_fpr_val, sizeof(freg_t));
  *length = sizeof(freg_t);

  return 0;
}

int simlib_t::read_fpr(uint32_t procid, uint64_t index, uint8_t* value, uint32_t* length)
{
  //Check if the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index provided is valid
  if(index >= NFPR)
    return 3;

  if(*length < sizeof(freg_t))
    return 4;

  //All checks have passed
  freg_t temp_fpr_val = get_core(procid)->get_state()->FPR.read_no_callback(index);
  memcpy(value, &temp_fpr_val, sizeof(freg_t));
  *length = sizeof(freg_t);

  return 0;
}

int simlib_t::read_vecr(uint32_t procid, const std::string& input_name, uint8_t* value, uint32_t* length)
{
  //Check that the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the name provided is valid
  uint64_t index = get_vecr_number(input_name);
  if(index >= NVPR)
    return 3;

  //Check that the advertised length of the buffer in bytes can hold the requested data
  size_t vlen = get_core(procid)->VU.get_vlen() / 8;
  if(vlen > *length){
    return 4;
  }

  //All checks passed
  size_t elen = get_core(procid)->VU.get_elen() / 8;
  size_t num_elem = vlen/elen;

  //Write the elements into the value buffer
  for(size_t element = 0; element < num_elem; ++element)
  {
    uint64_t val = 0ull;
    switch(elen)
    {
      case 8:
      val = get_core(procid)->VU.elt<uint64_t>(index, element);
      break;
      case 4:
      val = get_core(procid)->VU.elt<uint32_t>(index, element);
      break;
      case 2:
      val = get_core(procid)->VU.elt<uint16_t>(index, element);
      break;
      case 1:
      val = get_core(procid)->VU.elt<uint8_t>(index, element);
      break;
    }

    memcpy(value + element * elen, &val, elen);
  }

  //Set the length value to the number of bytes that was written
  *length = vlen;

  return 0;
}

int simlib_t::read_vecr(uint32_t procid, uint64_t index, uint8_t* value, uint32_t* length)
{
  //Check that the pointers point to something
  if(value == nullptr || length == nullptr)
    return 1;

  //Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index provided is valid
  if(index >= NVPR)
    return 3;

  //Check that the advertised length of the buffer in bytes can hold the requested data
  size_t vlen = get_core(procid)->VU.get_vlen() / 8;
  if(vlen > *length){
    return 4;
  }

  //All checks passed
  size_t elen = get_core(procid)->VU.get_elen() / 8;
  size_t num_elem = vlen/elen;

  //Write the elements into the value buffer
  for(size_t element = 0; element < num_elem; ++element)
  {
    uint64_t val = 0ull;
    switch(elen)
    {
      case 8:
      val = get_core(procid)->VU.elt<uint64_t>(index, element);
      break;
      case 4:
      val = get_core(procid)->VU.elt<uint32_t>(index, element);
      break;
      case 2:
      val = get_core(procid)->VU.elt<uint16_t>(index, element);
      break;
      case 1:
      val = get_core(procid)->VU.elt<uint8_t>(index, element);
      break;
    }

    memcpy(value + element * elen, &val, elen);
  }

  //Set the length value to the number of bytes that was written
  *length = vlen;

  return 0;
}

int simlib_t::partial_read_vecr(uint32_t procid, uint64_t index, uint8_t* pValue, uint32_t length, uint32_t offset)
{
  //Check that the pointers point to something
  if(pValue == nullptr)
    return 1;

  //Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index provided is valid
  if(index >= NVPR)
    return 3;

  //Check that the advertised length of the buffer in bytes can hold the requested data
  size_t vlen = get_core(procid)->VU.get_vlen() / 8;
  if(vlen < (offset + length)){
    return 4;
  }

  //All checks passed; write the elements into the value buffer
  // make this a memcpy
  for(size_t element = offset; element < (offset + length); ++element)
  {
    pValue[element - offset] = get_core(procid)->VU.elt<uint8_t>(index, element);
  }

  return 0;
}


int simlib_t::partial_write_vecr(uint32_t procid, uint64_t index, const uint8_t* pValue, uint32_t length, uint32_t offset)
{
  //Check that the pointers point to something
  if(pValue == nullptr)
    return 1;

  //Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index provided is valid
  if(index >= NVPR)
    return 3;

  //Check that the advertised length of the buffer in bytes can hold the requested data
  size_t vlen = get_core(procid)->VU.get_vlen() / 8;
  if(vlen < (offset + length)){
    return 4;
  }

  //All checks passed; write the elements into the value buffer
  // make this a memcpy
  for(size_t element = offset; element < (offset + length); ++element)
  {
    get_core(procid)->VU.elt<uint8_t>(index, element) = pValue[element - offset];
  }

  return 0;
}


int simlib_t::write_csr(uint32_t procid, uint64_t index, const uint64_t* value, uint32_t length)
{
  //Check if the pointers point to something
  if(value == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index is valid
  if(index >= NCSR)
    return 3;

  //Is the indended write length matching the xlen value?
  if(length != (get_core(procid)->get_xlen() / 8))
    return 4;

  //All checks pass, so go ahead and and write to the csr
  get_core(procid)->put_csr_api(index, *value);

  return 0;
}

int simlib_t::write_csr(uint32_t procid, const std::string& input_name, const uint64_t* value, uint32_t length)
{
  //Check if the pointers point to something
  if(value == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the name is valid
  uint64_t index = get_csr_number(input_name);
  if(index >= NCSR)
    return 3;

  //Is the indended write length matching the xlen value?
  if(length != (get_core(procid)->get_xlen() / 8))
    return 4;

  //All checks pass, so go ahead and and write to the csr
  get_core(procid)->put_csr_api(index, *value);

  return 0;
}

int simlib_t::write_xpr(uint32_t procid, const std::string& input_name, const uint64_t* value, uint32_t length)
{
  //Check if the pointers point to something
  if(value == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the name is valid
  uint64_t index = get_xpr_number(input_name);
  if(index >= NXPR)
    return 3;

  //Is the indended write length matching the xlen value?
  //if(length != (get_core(procid)->get_xlen() / 8))
  //  return 4;

  //All checks pass, so go ahead and load the value buffer and set the length to indicate the used bytes in the value buffer
  get_core(procid)->get_state()->XPR.write_no_callback(index, *value);

  return 0;
}

int simlib_t::write_xpr(uint32_t procid, uint64_t index, const uint64_t* value, uint32_t length)
{
  //Check if the pointers point to something
  if(value == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //check if the index provided is valid
  if(index >= NXPR)
    return 3;

  //Is the indended write length matching the xlen value?
  //if(length != (get_core(procid)->get_xlen() / 8))
  //  return 4;

  //All checks pass so we're go to write
  get_core(procid)->get_state()->XPR.write_no_callback(index, *value);

  return 0;
}

int simlib_t::write_fpr(uint32_t procid, const std::string& input_name, const uint8_t* value, uint32_t length)
{
  //Check if the pointers point to something
  if(value == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //check if the name provided is valid
  uint64_t index = get_fpr_number(input_name);
  if(index >= NFPR)
    return 3;

  //Is the indended write length matching the flen value?
  if(length > sizeof(freg_t))
    return 4;

  //All checks have passed perform the write via a memory operation into the floating point register model
  freg_t temp_fpr_val;
  temp_fpr_val.v[0] = 0xffffffffffffffffull;
  temp_fpr_val.v[1] = 0xffffffffffffffffull;
  memcpy(&temp_fpr_val, value, length);
  get_core(procid)->get_state()->FPR.write_no_callback(index, temp_fpr_val);

  return 0;
}

int simlib_t::write_fpr(uint32_t procid, uint64_t index, const uint8_t* value, uint32_t length)
{
  //Check if the pointers point to something
  if(value == nullptr)
    return 1;

  //Check if the procid AKA hart id is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index provided is valid
  if(index >= NFPR)
    return 3;

  //Is the indended write length matching the flen value?
  if(length > sizeof(freg_t))
    return 4;

  //All checks have passed perform the write via a memory operation into the floating point register model
  freg_t temp_fpr_val;
  temp_fpr_val.v[0] = 0xffffffffffffffffull;
  temp_fpr_val.v[1] = 0xffffffffffffffffull;
  memcpy(&temp_fpr_val, value, length);
  get_core(procid)->get_state()->FPR.write_no_callback(index, temp_fpr_val);

  return 0;
}

//will probably have to cast differently here and other places to the get the pointer types the same and compatible with the function.
int simlib_t::write_vecr(uint32_t procid, const std::string& input_name, const uint8_t* value, uint32_t length)
{
  //Check that the pointers point to something
  if(value == nullptr)
    return 1;

  //Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the name provided is valid
  uint64_t index = get_vecr_number(input_name);
  if(index >= NVPR)
    return 3;

  //Check that the advertised length of the buffer in bytes equals the size of the vector register
  size_t vlen = get_core(procid)->VU.get_vlen() / 8;
  if(vlen != length)
    return 4;

  //All checks passed
  size_t elen = get_core(procid)->VU.get_elen() / 8;
  size_t num_elem = vlen/elen;

  //Write the elements into the value buffer
  for(size_t element = 0; element < num_elem; ++element)
  {
    switch(elen)
    {
      case 8:
      {
        uint64_t* reg = &(get_core(procid)->VU.elt<uint64_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
      case 4:
      {
        uint32_t* reg = &(get_core(procid)->VU.elt<uint32_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
      case 2:
      {
        uint16_t* reg = &(get_core(procid)->VU.elt<uint16_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
      case 1:
      {
        uint8_t* reg = &(get_core(procid)->VU.elt<uint8_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
    }

    // void *reg;
    // switch(elen)
    // {
    //   case 8:
    //   {
    //     reg = (void *)&(get_core(procid)->VU.elt<uint64_t>(index, element));
    //     break;
    //   }
    //   case 4:
    //   {
    //     reg = (void *)&(get_core(procid)->VU.elt<uint32_t>(index, element));
    //     memcpy(reg, value + element * elen, elen);
    //     break;
    //   }
    //   case 2:
    //   {
    //     reg = (void *)&(get_core(procid)->VU.elt<uint16_t>(index, element));
    //     break;
    //   }
    //   case 1:
    //   {
    //     reg = (void *)&(get_core(procid)->VU.elt<uint8_t>(index, element));
    //     break;
    //   }
    // } 

    // memcpy(reg, value + element * elen, elen);
  }

  return 0;
}

int simlib_t::write_vecr(uint32_t procid, uint64_t index, const uint8_t* value, uint32_t length)
{
  //Check that the pointers point to something
  if(value == nullptr)
    return 1;

  //Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  //Check if the index provided is valid
  if(index >= NVPR)
    return 3;

  //Check that the advertised length of the buffer in bytes can hold the requested data
  size_t vlen = get_core(procid)->VU.get_vlen() / 8;
  if(vlen != length)
    return 4;

  //All checks passed
  size_t elen = get_core(procid)->VU.get_elen() / 8;
  size_t num_elem = vlen/elen;

  //Write the elements into the value buffer
  for(size_t element = 0; element < num_elem; ++element) {
    switch(elen) {
      case 8:
      {
        uint64_t* reg = &(get_core(procid)->VU.elt<uint64_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
      case 4:
      {
        uint32_t* reg = &(get_core(procid)->VU.elt<uint32_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
      case 2:
      {
        uint16_t* reg = &(get_core(procid)->VU.elt<uint16_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
      case 1:
      {
        uint8_t* reg = &(get_core(procid)->VU.elt<uint8_t>(index, element));
        memcpy(reg, value + element * elen, elen);
        break;
      }
    }
  }

  return 0;
}

int simlib_t::translate_virtual_address_api(int procid, const uint64_t* vaddr, int intent, uint64_t* paddr, uint64_t* memattrs)
{
  // check the pointers do point to something
  if(vaddr == nullptr || paddr == nullptr || memattrs == nullptr)
    return 1;

  // Check that the procid AKA hart is valid
  if(not core_id_exist(procid))
    return 2;

  // get the mmu for the specified processor and call the translation api function
  access_type type;
  switch(intent)
  {
    case(0):
      type = access_type::LOAD;
      break;
    case(1):
      type = access_type::STORE;
      break;
    case(2):
      type = access_type::FETCH;
      break;
  }
  // TODO(Noah): Investigate what might need to be passed for the xlate_flags parameter when there
  // is time to do so.
  // length is set to 1 byte here because the api requirements were for individual bytes only.
  int status = get_core(procid)->get_mmu()->translate_api(*vaddr, paddr, memattrs, reg_t(1) /* length */, static_cast<access_type>(intent), 0);

  if(status == 0) {
    return 0;
  }

  return status + 2;
}

bool simlib_t::set_pc_api(int procid, const std::string& name, const uint8_t* bytes, size_t len)
{
  if(bytes == nullptr) {
    return false;
  } else if(not core_id_exist(procid)) {
    return false;
  }

  processor_t* proc = get_core(procid);
  if(proc == nullptr) {
    return false;
  }

  return get_core(procid)->set_pc_api(name, bytes, len);
}

bool simlib_t::get_pc_api(int procid, const std::string& name, uint8_t* bytes, size_t len)
{
  if(bytes == nullptr) {
    return false;
  } else if(not core_id_exist(procid)) {
    return false;
  }

  processor_t* proc = get_core(procid);
  if(proc == nullptr) {
    return false;
  }

  return get_core(procid)->get_pc_api(bytes, name, len); // The method called in processor_t will validate the name and length.
}

bool simlib_t::set_privilege_api(int procid, const uint64_t* val)
{
  if (nullptr == val) return false;
  if (!core_id_exist(procid)) return false;

  processor_t* proc = get_core(procid);
  if (nullptr == proc) return false;

  //assert in legalize_privilege should catch invalid values
  //register field is only 2 bits so shouldn't be able to pass invalid case from force
  proc->set_privilege_api(*val, proc->get_state()->v);
  return true;
}

bool simlib_t::get_privilege_api(int procid, uint64_t* val)
{
  if(nullptr == val) return false;
  if(!core_id_exist(procid)) return false;

  processor_t* proc = get_core(procid);
  if(nullptr == proc) return false;

  proc->get_privilege_api(val);
  return true;
}

#endif
