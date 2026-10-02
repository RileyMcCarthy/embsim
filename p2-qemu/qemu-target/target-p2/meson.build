# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy

# ONE insn.decode, TWO dispatchers. The hub-exec translator emits TCG through
# trans_*; the cog-exec interpreter executes through iexec_*. Generating both
# from the same table is what keeps them from disagreeing about an encoding --
# the same reason the table itself is generated from p2core's.
gen = [
  decodetree.process('insn.decode', extra_args: [ '--static-decode', 'decode_p2',
                                                  '--insnwidth', '32' ])
]

decode_interp = custom_target('decode-interp.c.inc',
  output: 'decode-interp.c.inc',
  input: 'insn.decode',
  command: [find_program(meson.project_source_root() / 'scripts' / 'decodetree.py'),
            '@INPUT@', '--static-decode', 'interp_p2', '--translate', 'iexec',
            '--insnwidth', '32', '-o', '@OUTPUT@'])

# A stub for every pattern each dispatcher does not hand-write (gen_stubs.py),
# generated from that dispatcher's own decoder into the build directory, where
# translate.c and interp.c #include them.
trans_stub = custom_target('trans_stub.c.inc',
  output: 'trans_stub.c.inc',
  input: gen,
  command: [python, files('gen_stubs.py'), '@INPUT@', '@OUTPUT@'])

interp_stub = custom_target('interp_stub.c.inc',
  output: 'interp_stub.c.inc',
  input: decode_interp,
  command: [python, files('gen_stubs.py'), '@INPUT@', '@OUTPUT@', '--interp'])

p2_ss = ss.source_set()
p2_system_ss = ss.source_set()

p2_ss.add(gen)
p2_ss.add(decode_interp)
p2_ss.add(trans_stub, interp_stub)
p2_ss.add(files('translate.c', 'op_helper.c', 'cpu.c', 'pinbus.c', 'interp.c',
                   'hostipc.c'))

target_arch += {'p2': p2_ss}
target_system_arch += {'p2': p2_system_ss}
