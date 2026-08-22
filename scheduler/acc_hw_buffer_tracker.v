// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Simon Davidson, University of Manchester
////////////////////////////////////////////////////////////////////////
//
// acc_hw_buffer_tracker
//
// Author: Simon D.
// Version: 2.1
// Date 10/2/2025
// Authors: Simon Davidson & Claude   Last modified: 2026-08-22
//
// Tracks which buffers are in use by one hardware accelerator.
// Stores six slots (mode + buffer ID + #targets) so sch_buffer_state
// can correctly update buffer state on task completion.
//
// Slot modes:
//   2'b00 unused  — ignored on completion
//   2'b01 source  — no action on dispatch; consumed (usage count decremented) on completion
//   2'b10 rw      — claimed on dispatch, refilled on completion
//   2'b11 target  — reserved on dispatch, filled on completion
//
// DEPTH (2026-08-22)
// -----------------
// DEPTH=1 (default) is the original design, kept VERBATIM in its own generate
// branch: exactly one task in flight per accelerator, `acc_free_o` low from
// dispatch to completion.  Bosch / FMI / every existing top get this and are
// bit-identical.
//
// DEPTH=2 lets an accelerator accept its NEXT task while the CURRENT one is
// still draining its output stage — used by the JABRA/Monarch annAcc, whose
// neuron_processing tail is ~10% of an nblocks=40 frame.  Two slot sets are
// held in a 2-deep FIFO so the completing (oldest) task's buffer bookkeeping
// survives the early dispatch of its successor; without this the single slot
// register is overwritten and sch_buffer_state frees/fills the WRONG buffers.
//
// With DEPTH=2 the accelerator must drive `acc_ready_next_i` — a level meaning
// "my input stages are idle, only the output stage is draining".  It is ignored
// entirely when DEPTH=1.  Accepting a second task ALSO requires the caller to
// keep the data-dependency checks intact (buffer full/free/colour still gate
// dispatch in sch_entry, and still update on completion), so relaxing this is a
// RESOURCE relaxation only.
//

module acc_hw_buffer_tracker
    #(parameter NUM_SLOTS    = 6,
      parameter BUFF_INDX_SZ = 4,
      parameter TGT_COUNT_SZ = 3,
      parameter MODE_SZ      = 2,
      // 1 = original single in-flight task.  2 = allow one further task to be
      // accepted while the current one drains (needs acc_ready_next_i).
      parameter DEPTH        = 1)
    (input wire                                   clk,
     input wire                                   reset,

     input wire                                   new_task_i,
     input wire [NUM_SLOTS*BUFF_INDX_SZ-1:0]     slot_buff_i,
     input wire [NUM_SLOTS*MODE_SZ-1:0]           slot_mode_i,
     input wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]      slot_ntgt_i,

     input wire                                   task_finished_i,

     // DEPTH>1 only: accelerator says it can start another task now.  Tie low
     // (or leave at DEPTH=1) for the original behaviour.
     input wire                                   acc_ready_next_i,

     output wire                                  acc_free_o,
     output wire [NUM_SLOTS*BUFF_INDX_SZ-1:0]    slot_buff_o,
     output wire [NUM_SLOTS*MODE_SZ-1:0]          slot_mode_o,
     output wire [NUM_SLOTS*TGT_COUNT_SZ-1:0]     slot_ntgt_o
    );

localparam SLOTSET_SZ = NUM_SLOTS*(BUFF_INDX_SZ + MODE_SZ + TGT_COUNT_SZ);

generate
if (DEPTH < 2) begin : gen_depth1

   //--------------------------------------------------------------
   // Original single-entry tracker — unchanged.
   //--------------------------------------------------------------
   reg [NUM_SLOTS*BUFF_INDX_SZ-1:0]  slot_buff_r;
   reg [NUM_SLOTS*MODE_SZ-1:0]        slot_mode_r;
   reg [NUM_SLOTS*TGT_COUNT_SZ-1:0]   slot_ntgt_r;
   reg                                 acc_free_r;

   always @ (posedge clk)
   begin
      if (reset)
      begin
         slot_buff_r <= 'b0;
         slot_mode_r <= 'b0;
         slot_ntgt_r <= 'b0;
      end
      else if (new_task_i)
      begin
         slot_buff_r <= slot_buff_i;
         slot_mode_r <= slot_mode_i;
         slot_ntgt_r <= slot_ntgt_i;
      end
   end

   assign acc_free_o   = acc_free_r;
   assign slot_buff_o  = slot_buff_r;
   assign slot_mode_o  = slot_mode_r;
   assign slot_ntgt_o  = slot_ntgt_r;

   wire acc_free_nxt = new_task_i      ? 1'b0
                     : task_finished_i ? 1'b1
                     :                   acc_free_r;

   always @ (posedge clk)
   begin
      if (reset)
         acc_free_r <= 1'b1;
      else
         acc_free_r <= acc_free_nxt;
   end

end else begin : gen_depth2

   //--------------------------------------------------------------
   // Two-deep in-order FIFO of slot sets.
   //
   // ent_r[0] is the HEAD — the oldest in-flight task, i.e. the one the next
   // task_finished_i belongs to, and the one whose slots are published on
   // slot_*_o for sch_buffer_state's completion pass.  Accelerators complete
   // strictly in order, so a plain shift register is sufficient; no tags.
   //--------------------------------------------------------------
   reg [SLOTSET_SZ-1:0] ent_r [0:1];
   reg            [1:0] occ_r;          // 0, 1 or 2 tasks in flight
   reg                  acc_free_r;

   wire [SLOTSET_SZ-1:0] new_set = {slot_ntgt_i, slot_mode_i, slot_buff_i};

   wire push = new_task_i;
   wire pop  = task_finished_i;

   // Occupancy after this cycle's push/pop.  A simultaneous push+pop is legal
   // and leaves the count unchanged (head retires as the successor arrives).
   wire [1:0] occ_nxt = occ_r + {1'b0, push} - {1'b0, pop};

   always @ (posedge clk)
   begin
      if (reset)
      begin
         ent_r[0] <= 'b0;
         ent_r[1] <= 'b0;
         occ_r    <= 2'd0;
      end
      else
      begin
         occ_r <= occ_nxt;
         // Retire the head first, then land any new set at the tail.  Writing
         // both in one pass keeps push+pop in the same cycle correct.
         case ({push, pop})
            2'b01: begin                       // pop only
                      // Only shift when a successor is actually waiting.  On the
                      // last pop the head is HELD, exactly as the DEPTH=1
                      // register does, so the two are indistinguishable whenever
                      // tasks never overlap (tb_acc_tracker_d2 T4 asserts this).
                      // Consumers read slot_*_o on the completion cycle, when the
                      // head is still valid either way — but matching outright
                      // beats relying on that.
                      if (occ_r == 2'd2) ent_r[0] <= ent_r[1];
                   end
            2'b10: begin                       // push only
                      if (occ_r == 2'd0) ent_r[0] <= new_set;
                      else               ent_r[1] <= new_set;
                   end
            2'b11: begin                       // push + pop
                      if (occ_r == 2'd0)      ent_r[0] <= new_set;
                      else if (occ_r == 2'd1) ent_r[0] <= new_set;
                      else begin
                         ent_r[0] <= ent_r[1];
                         ent_r[1] <= new_set;
                      end
                   end
            default: ;
         endcase
      end
   end

   assign slot_buff_o = ent_r[0][0                                  +: NUM_SLOTS*BUFF_INDX_SZ];
   assign slot_mode_o = ent_r[0][NUM_SLOTS*BUFF_INDX_SZ             +: NUM_SLOTS*MODE_SZ];
   assign slot_ntgt_o = ent_r[0][NUM_SLOTS*(BUFF_INDX_SZ + MODE_SZ) +: NUM_SLOTS*TGT_COUNT_SZ];

   // Registered, exactly like DEPTH=1, so the dispatch cadence (one launch per
   // cycle, availability visible the cycle after it changes) is unchanged.
   // Empty  -> always free.
   // One    -> free only while the accelerator says its input stages are idle.
   // Two    -> full.
   wire acc_free_nxt = (occ_nxt == 2'd0) ? 1'b1
                     : (occ_nxt == 2'd1) ? acc_ready_next_i
                     :                     1'b0;

   always @ (posedge clk)
   begin
      if (reset)
         acc_free_r <= 1'b1;
      else
         acc_free_r <= acc_free_nxt;
   end

   assign acc_free_o = acc_free_r;

end
endgenerate

endmodule
