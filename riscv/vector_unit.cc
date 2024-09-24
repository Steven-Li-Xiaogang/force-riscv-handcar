// See LICENSE for license details

#include "config.h"
#include "vector_unit.h"
#include "processor.h"
#include "arith.h"

#ifdef FORCE_RISCV_ENABLE
extern "C"{
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

  // update_vector_element function: for the given cpuid, this callback function is called by the simulator to notify the user that a vector register element has been read or written
  //
  //  inputs:
  //      uint32_t cpuid -- refers to the processor ID
  //      const char* pRegName -- the base name of the vector register does NOT include a suffix for physical register since this is a FORCE / hardware specific notion.
  //      uint32_t vecRegIndex -- the numerical index that goes with the vector register base name
  //      uint32_t eltIndex -- the numerical index of the element that is updated
  //      uint32_t eltByteWidth -- the number of bytes per element at the time of the update, used in FORCE with the eltIndex to dynamically associate physical registers for aggregated updates
  //      const uint8_t* value -- the contents of the ENTIRE vector register if this update is a "read" or *nothing* if this is a "write".
  //      uint32_t byteLength -- should match the size of the ENTIRE vector register.
  //      const char* pAccessType -- should be "read" or "write".
  //
  void update_vector_element(uint32_t cpuid, const char *pRegName, uint32_t vecRegIndex, uint32_t eltIndex, uint32_t eltByteWidth, const uint8_t* pValue, uint32_t  byteLength, const char* pAccessType);
}

extern const char* vr_name[];

template<class T>
void vectorUnit_t::do_callback(reg_t vecRegIndex, reg_t eltIndex, const char pAccessType[]) const
{
  reg_t elts_per_reg = (VLEN >> 3) / sizeof(T);
  reg_t corrected_vreg_index = vecRegIndex + eltIndex / elts_per_reg;
  if (corrected_vreg_index > vecRegIndex)
    eltIndex %= elts_per_reg;

  #ifdef WORDS_BIGENDIAN
  // "V" spec 0.7.1 requires lower indices to map to lower significant
  // bits when changing SEW, thus we need to index from the end on BE.
  eltIndex ^= elts_per_reg - 1;
  #endif

  uint8_t *p_reg_start = (uint8_t*)((char*)reg_file + corrected_vreg_index * (VLEN >> 3));
  update_vector_element(p->get_state()->pid, vr_name[corrected_vreg_index], corrected_vreg_index, eltIndex, sizeof(T), p_reg_start, (VLEN >> 3), pAccessType);
}
#endif

void vectorUnit_t::reset()
{
  free(reg_file);
  VLEN = get_vlen();
  ELEN = get_elen();
  reg_file = malloc(NVPR * vlenb);
  memset(reg_file, 0, NVPR * vlenb);

  auto& csrmap = p->get_state()->csrmap;
  csrmap[CSR_VXSAT] = vxsat = std::make_shared<vxsat_csr_t>(p, CSR_VXSAT);
  csrmap[CSR_VSTART] = vstart = std::make_shared<vector_csr_t>(p, CSR_VSTART, /*mask*/ VLEN - 1);
  csrmap[CSR_VXRM] = vxrm = std::make_shared<vector_csr_t>(p, CSR_VXRM, /*mask*/ 0x3ul);
  csrmap[CSR_VL] = vl = std::make_shared<vector_csr_t>(p, CSR_VL, /*mask*/ 0);
  csrmap[CSR_VTYPE] = vtype = std::make_shared<vector_csr_t>(p, CSR_VTYPE, /*mask*/ 0);
  csrmap[CSR_VLENB] = std::make_shared<vector_csr_t>(p, CSR_VLENB, /*mask*/ 0, /*init*/ vlenb);
  assert(VCSR_VXSAT_SHIFT == 0);  // composite_csr_t assumes vxsat begins at bit 0
  csrmap[CSR_VCSR] = std::make_shared<composite_csr_t>(p, CSR_VCSR, vxrm, vxsat, VCSR_VXRM_SHIFT);

  vtype->write_raw(0);
#ifdef FORCE_RISCV_ENABLE
  update_generator_register(p->get_id(), "vtype", vtype->read(), 0xffffffffffffffff, "write");
#endif
  set_vl(0, 0, 0, -1); // default to illegal configuration
}

reg_t vectorUnit_t::set_vl(int rd, int rs1, reg_t reqVL, reg_t newType)
{
  int new_vlmul = 0;
  if (vtype->read() != newType) {
    vsew = 1 << (extract64(newType, 3, 3) + 3);
    new_vlmul = int8_t(extract64(newType, 0, 3) << 5) >> 5;
    vflmul = new_vlmul >= 0 ? 1 << new_vlmul : 1.0 / (1 << -new_vlmul);
    vlmax = (VLEN/vsew) * vflmul;
    vta = extract64(newType, 6, 1);
    vma = extract64(newType, 7, 1);

    vill = !(vflmul >= 0.125 && vflmul <= 8)
           || vsew > std::min(vflmul, 1.0f) * ELEN
           || (newType >> 8) != 0;

    if (vill) {
      vlmax = 0;
      vtype->write_raw(UINT64_MAX << (p->get_xlen() - 1));
    } else {
      vtype->write_raw(newType);
    }
#ifdef FORCE_RISCV_ENABLE
  update_generator_register(p->get_id(), "vtype", vtype->read(), 0xffffffffffffffff, "write");
#endif
  }

  // set vl
  if (vlmax == 0) {
    vl->write_raw(0);
  } else if (rd == 0 && rs1 == 0) {
    vl->write_raw(std::min(vl->read(), vlmax));
  } else if (rd != 0 && rs1 == 0) {
    vl->write_raw(vlmax);
  } else if (rs1 != 0) {
    vl->write_raw(std::min(reqVL, vlmax));
  }

#ifdef FORCE_RISCV_ENABLE
  update_generator_register(p->get_id(), "vl", vl->read(), 0xffffffffffffffff, "write");
#endif
  vstart->write_raw(0);
#ifdef FORCE_RISCV_ENABLE
  update_generator_register(p->get_id(), "vstart", vstart->read(), 0xffffffffffffffff, "write");
#endif
  setvl_count++;
  return vl->read();
}

#ifdef FORCE_RISCV_ENABLE
reg_t vectorUnit_t::set_vl_api(reg_t reqVL, reg_t newType){
  int new_vlmul = 0;
  if (vtype->read() != newType){
    vtype->write_raw(newType);
    vsew = 1 << (extract64(newType, 3, 3) + 3);
    new_vlmul = int8_t(extract64(newType, 0, 3) << 5) >> 5;
    vflmul = new_vlmul >= 0 ? 1 << new_vlmul : 1.0 / (1 << -new_vlmul);
    vlmax = (VLEN/vsew) * vflmul;
    vta = extract64(newType, 6, 1);
    vma = extract64(newType, 7, 1);

    vill = !(vflmul >= 0.125 && vflmul <= 8)
           || vsew > std::min(vflmul, 1.0f) * ELEN
           || (newType >> 8) != 0;

    if (vill) {
      vlmax = 0;
      vtype->write_raw(UINT64_MAX << (p->get_xlen() - 1));
    }
  }

  /* different from set_vl */
  vl->write_raw(reqVL);
  vstart->write_raw(0);
  return vl->read();
}
#endif

template<class T> T& vectorUnit_t::elt(reg_t vRegIndex, reg_t n, bool UNUSED is_write) {
  assert(vsew != 0);
  assert((VLEN >> 3)/sizeof(T) > 0);
  reg_t elts_per_reg = (VLEN >> 3) / (sizeof(T));
  vRegIndex += n / elts_per_reg;
  n = n % elts_per_reg;
#ifdef WORDS_BIGENDIAN
  // "V" spec 0.7.1 requires lower indices to map to lower significant
  // bits when changing SEW, thus we need to index from the end on BE.
  n ^= elts_per_reg - 1;
#endif

#ifndef FORCE_RISCV_ENABLE
  reg_referenced[vRegIndex] = 1;

  if (unlikely(p->get_log_commits_enabled() && is_write))
    p->get_state()->log_reg_write[((vRegIndex) << 4) | 2] = {0, 0};
#endif

  T *regStart = (T*)((char*)reg_file + vRegIndex * (VLEN >> 3));
  return regStart[n];
}

#ifdef FORCE_RISCV_ENABLE
template<class T> T& vectorUnit_t::elt_do_callback(reg_t vRegIndex, reg_t n, bool UNUSED is_write) {
  T& reg_val = elt<T>(vRegIndex, n, is_write);
  do_callback<T>(vRegIndex, n, is_write ? "write" : "read");
  return reg_val;
}
#endif

// The logic differences between 'elt()' and 'elt_group()' come from
// the fact that, while 'elt()' requires that the element is fully
// contained in a single vector register, the element group may span
// multiple registers in a single register group (LMUL>1).
//
// Notes:
// - We do NOT check that a single element - i.e., the T in the element
//   group type std::array<T, N> - fits within a single register, or that
//   T is smaller or equal to VSEW. Implementations of the instructions
//   sometimes use a different T than what the specification suggests.
//   Instructon implementations should 'require()' what the specification
//   dictates.
// - We do NOT check that 'vRegIndex' is a valid register group, or that
//   'n+1' element groups fit in the register group 'vRegIndex'. It is
//   the responsibility of the caller to validate those preconditions.
template<typename EG> EG&
vectorUnit_t::elt_group(reg_t vRegIndex, reg_t n, bool UNUSED is_write) {
#ifdef WORDS_BIGENDIAN
  fputs("vectorUnit_t::elt_group is not compatible with WORDS_BIGENDIAN setup.\n",
          stderr);
  abort();
#endif
  using T = typename EG::value_type;
  constexpr std::size_t N = std::tuple_size<EG>::value;
  assert(N > 0);

  assert(vsew != 0);
  constexpr reg_t elt_group_size = N * sizeof(T);
  const reg_t reg_group_size = (VLEN >> 3) * vflmul;
  assert(((n + 1) * elt_group_size) <= reg_group_size);

  const reg_t start_byte = n * elt_group_size;
  const reg_t bytes_per_reg = VLEN >> 3;

  // Inclusive first/last register indices.
  const reg_t reg_first = vRegIndex + start_byte / bytes_per_reg;
  const reg_t reg_last = vRegIndex + (start_byte + elt_group_size - 1) / bytes_per_reg;

#ifndef FORCE_RISCV_ENABLE
  // Element groups per register groups
  for (reg_t vidx = reg_first; vidx <= reg_last; ++vidx) {
      reg_referenced[vidx] = 1;

      if (unlikely(p->get_log_commits_enabled() && is_write)) {
          p->get_state()->log_reg_write[(vidx << 4) | 2] = {0, 0};
      }
  }
#endif

  return *(EG*)((char*)reg_file + vRegIndex * (VLEN >> 3) + start_byte);
}

template signed char& vectorUnit_t::elt<signed char>(reg_t, reg_t, bool);
template short& vectorUnit_t::elt<short>(reg_t, reg_t, bool);
template int& vectorUnit_t::elt<int>(reg_t, reg_t, bool);
template long& vectorUnit_t::elt<long>(reg_t, reg_t, bool);
template long long& vectorUnit_t::elt<long long>(reg_t, reg_t, bool);
template uint8_t& vectorUnit_t::elt<uint8_t>(reg_t, reg_t, bool);
template uint16_t& vectorUnit_t::elt<uint16_t>(reg_t, reg_t, bool);
template uint32_t& vectorUnit_t::elt<uint32_t>(reg_t, reg_t, bool);
template uint64_t& vectorUnit_t::elt<uint64_t>(reg_t, reg_t, bool);
template float16_t& vectorUnit_t::elt<float16_t>(reg_t, reg_t, bool);
template float32_t& vectorUnit_t::elt<float32_t>(reg_t, reg_t, bool);
template float64_t& vectorUnit_t::elt<float64_t>(reg_t, reg_t, bool);

#ifdef FORCE_RISCV_ENABLE
template signed char& vectorUnit_t::elt_do_callback<signed char>(reg_t, reg_t, bool);
template short& vectorUnit_t::elt_do_callback<short>(reg_t, reg_t, bool);
template int& vectorUnit_t::elt_do_callback<int>(reg_t, reg_t, bool);
template long& vectorUnit_t::elt_do_callback<long>(reg_t, reg_t, bool);
template long long& vectorUnit_t::elt_do_callback<long long>(reg_t, reg_t, bool);
template uint8_t& vectorUnit_t::elt_do_callback<uint8_t>(reg_t, reg_t, bool);
template uint16_t& vectorUnit_t::elt_do_callback<uint16_t>(reg_t, reg_t, bool);
template uint32_t& vectorUnit_t::elt_do_callback<uint32_t>(reg_t, reg_t, bool);
template uint64_t& vectorUnit_t::elt_do_callback<uint64_t>(reg_t, reg_t, bool);
template float16_t& vectorUnit_t::elt_do_callback<float16_t>(reg_t, reg_t, bool);
template float32_t& vectorUnit_t::elt_do_callback<float32_t>(reg_t, reg_t, bool);
template float64_t& vectorUnit_t::elt_do_callback<float64_t>(reg_t, reg_t, bool);
#endif

template EGU32x4_t& vectorUnit_t::elt_group<EGU32x4_t>(reg_t, reg_t, bool);
template EGU32x8_t& vectorUnit_t::elt_group<EGU32x8_t>(reg_t, reg_t, bool);
template EGU64x4_t& vectorUnit_t::elt_group<EGU64x4_t>(reg_t, reg_t, bool);
template EGU8x16_t& vectorUnit_t::elt_group<EGU8x16_t>(reg_t, reg_t, bool);
