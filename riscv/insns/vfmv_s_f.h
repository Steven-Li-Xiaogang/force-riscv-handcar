// vfmv_s_f: vd[0] = rs1 (vs2=0)
require_vector(true);
require_fp;
require((P.VU.vsew == e16 && p->extension_enabled(EXT_ZVFH)) ||
        (P.VU.vsew == e32 && p->extension_enabled('F')) ||
        (P.VU.vsew == e64 && p->extension_enabled('D')));
require(STATE.frm->read() < 0x5);

reg_t vl = P.VU.vl->read();

if (vl > 0 && P.VU.vstart->read() < vl) {
  reg_t rd_num = insn.rd();

#ifndef FORCE_RISCV_ENABLE
  switch (P.VU.vsew) {
    case e16:
      P.VU.elt<uint16_t>(rd_num, 0, true) = f16(FRS1).v;
      break;
    case e32:
      P.VU.elt<uint32_t>(rd_num, 0, true) = f32(FRS1).v;
      break;
    case e64:
      if (FLEN == 64)
        P.VU.elt<uint64_t>(rd_num, 0, true) = f64(FRS1).v;
      else
        P.VU.elt<uint64_t>(rd_num, 0, true) = f32(FRS1).v;
      break;
  }
#else
  switch (P.VU.vsew) {
    case e16:
      P.VU.elt_do_callback<uint16_t>(rd_num, 0, true) = f16(FRS1).v;
      break;
    case e32:
      P.VU.elt_do_callback<uint32_t>(rd_num, 0, true) = f32(FRS1).v;
      break;
    case e64:
      if (FLEN == 64)
        P.VU.elt_do_callback<uint64_t>(rd_num, 0, true) = f64(FRS1).v;
      else
        P.VU.elt_do_callback<uint64_t>(rd_num, 0, true) = f32(FRS1).v;
      break;
  }
#endif
}
P.VU.vstart->write(0);
