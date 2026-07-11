# CUBS2 lockstep adapter

This directory contains only CUBS2-owned integration: the vehicle's generated
Synapse payload layout, native host mapping, and the exported
`cubs2_fastdyn_lockstep_shared` storage used by FastDyn.

The reusable sequencing implementation lives in the `cerebri_modules` west
module and is consumed through `<cerebri_lockstep/sequence.h>`. Do not duplicate
generation, wait/respond, termination, or memory-ordering logic here.
