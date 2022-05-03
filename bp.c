/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bp_api.h"

// FSM states
#define STRONGLY_NOT_TAKEN 0
#define WEAKLY_NOT_TAKEN 1
#define WEAKLY_TAKEN 2
#define STRONGLY_TAKEN 3

// share status options
#define NOT_USING_SHARE 0
#define USING_SHARE_LSB 1
#define USING_SHARE_MID 2

#define COMMAND_SIZE 32

// Every entry in btb will hold those values, some will be irrelevant, base on config.
struct btb_entry{
    int tag;
    uint32_t target;
    int history;
    unsigned* result_table;
    bool valid;
};

// Global btb table.
struct btb_entry* btb_table;

// Global params we wanna keep.
int result_table_size;
int share_status;
int btb_size;
unsigned* global_result_table;
unsigned int global_history = 0;
bool is_global_hist;
bool is_global_table;
unsigned default_state;
unsigned tag_size;

// Global values we return with stats function.
int num_of_updates = 0;
int num_of_flushes = 0;
int theoretical_memory_size;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){

    // We keep params as global and allocate the global  b2b table.
    btb_size = btbSize;
    btb_table = (struct btb_entry*)malloc(btbSize * sizeof(struct btb_entry));
    result_table_size = pow(2,historySize);
    share_status = Shared;
    is_global_hist = isGlobalHist;
    is_global_table = isGlobalTable;
    default_state = fsmState;
    tag_size = tagSize;

    // All the btb entries are invalid at init.
    for (int i = 0; i < btbSize; ++i) {
        btb_table[i].valid = false;
    }

    // Local history, local table
    if(!isGlobalHist && !isGlobalTable) {
        for (int i = 0; i < btbSize; ++i) {
            btb_table[i].history = 0;
            btb_table[i].valid = false;
            btb_table[i].result_table = (unsigned*)malloc(result_table_size * sizeof(unsigned));
            for (int j = 0; j < result_table_size; ++j) {
                btb_table[i].result_table[j] = fsmState;
            }
        }
        theoretical_memory_size = btbSize*(1 + tagSize + (COMMAND_SIZE - 2) + historySize + 2*result_table_size);
    }
    // Local history, global table
    else if(!isGlobalHist && isGlobalTable){
        global_result_table = (unsigned*)malloc(result_table_size * sizeof(unsigned));
        for (int j = 0; j < result_table_size; ++j) {
            global_result_table[j] = fsmState;
        }
        theoretical_memory_size = btbSize*(1 + tagSize + (COMMAND_SIZE - 2) + historySize) + 2*result_table_size;
    }
    // global history, global table
    else if(isGlobalHist && isGlobalTable){
        global_result_table = (unsigned*)malloc(result_table_size * sizeof(unsigned));
        for (int j = 0; j < result_table_size; ++j) {
            global_result_table[j] = fsmState;
        }
        theoretical_memory_size = btbSize*(1 + tagSize + (COMMAND_SIZE - 2)) + historySize + 2*result_table_size;
    }
    // global history, local table
    else if(isGlobalHist && !isGlobalTable){
        for (int i = 0; i < btbSize; ++i) {
            btb_table[i].result_table = (unsigned*)malloc(result_table_size * sizeof(unsigned));
            for (int j = 0; j < result_table_size; ++j) {
                btb_table[i].result_table[j] = fsmState;
            }
        }
        theoretical_memory_size = btbSize*(1 + tagSize + (COMMAND_SIZE - 2) + 2*result_table_size) + historySize;
    }

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

    // We calc the b2b index.
    unsigned int calc_pc = pc >> 2;
    unsigned int index = calc_pc % btb_size;

    // We calc the tag.
    unsigned int b2b_bits =  (unsigned int)log2(btb_size);
    unsigned int tag_calc_pc = calc_pc >> b2b_bits;
    unsigned int tag = tag_calc_pc % (unsigned int)pow(2, tag_size);

    // If there is a valid entry, we check the table, else return false.
    if(btb_table[index].valid == true && btb_table[index].tag == tag){

        // Determine history.
        unsigned int current_hist;
        if(is_global_hist){
            current_hist = global_history;
        }else{
            current_hist = btb_table[index].history;
        }

        // Determine table;
        unsigned* current_result_table;
        if(is_global_table){
            current_result_table = global_result_table;
        }else{
            current_result_table = btb_table[index].result_table;
        }

        // Calc the fsm table index.
        int access_index;
        if(is_global_table && share_status == USING_SHARE_LSB){
            unsigned int calc_pc_lsb = pc >> 2;
            access_index =  (calc_pc_lsb ^ current_hist) % result_table_size;
        }else if(is_global_table && share_status == USING_SHARE_MID){
            unsigned int calc_pc_mid = pc >> 16;
            access_index =  (calc_pc_mid ^ current_hist) % result_table_size;
        }else{
            access_index = current_hist % result_table_size;
        }

        // access the fsm table.
        unsigned result = current_result_table[access_index];

        // If the result is WT or ST, return true and update target. else return false.
        if(result == WEAKLY_TAKEN || result == STRONGLY_TAKEN){
            *dst = btb_table[index].target;
            return true;
        }else{
            *dst = pc + 4;
            return false;
        }

    }else{
        *dst = pc + 4;
        return false;
    }
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){

    num_of_updates++;

    // We kush if misspredict earlier.
    if((taken && (pred_dst == pc+4)) || (!taken && pred_dst == targetPc)){
        num_of_flushes++;
    }

    unsigned int calc_pc = pc >> 2;
    unsigned int index = calc_pc % btb_size;
    unsigned int b2b_bits =  (unsigned int)log2(btb_size);
    unsigned int tag_calc_pc = calc_pc >> b2b_bits;
    unsigned int tag = tag_calc_pc % (unsigned int)pow(2, tag_size);

    // If the new entry access an invalid btb entry or one with a different tag, we overwrite.
    if(btb_table[index].valid == false || btb_table[index].tag != tag){

        btb_table[index].valid = true;
        btb_table[index].tag = tag;
        btb_table[index].target = targetPc;

        int current_hist;
        if(!is_global_hist){
            // If it's local history and overwrite, we change history to 0.
            btb_table[index].history = 0;
            current_hist = btb_table[index].history;
        }else{
            current_hist = global_history;
        }

        if(!is_global_table){
            // If local fsm table, we default the table values.
            for (int i = 0; i < result_table_size; ++i) {
                btb_table[index].result_table[i] = default_state;
            }
        }

        int access_index;
        if(is_global_table && share_status == USING_SHARE_LSB){
            unsigned int calc_pc_lsb = pc >> 2;
            access_index =  (calc_pc_lsb ^ current_hist) % result_table_size;
        }else if(is_global_table && share_status == USING_SHARE_MID){
            unsigned int calc_pc_mid = pc >> 16;
            access_index =  (calc_pc_mid ^ current_hist) % result_table_size;
        }else{
            access_index = current_hist % result_table_size;
        }

        unsigned* current_result_table;
        if(is_global_table){
            current_result_table = global_result_table;
        }else{
            current_result_table = btb_table[index].result_table;
        }

        // We change the fsm state based on the branch taken.
        if(taken && (current_result_table[access_index] != STRONGLY_TAKEN)){
            current_result_table[access_index]++;
        }else if(!taken && (current_result_table[access_index] != STRONGLY_NOT_TAKEN)){
            current_result_table[access_index]--;
        }

        // We update the history.
        if(!is_global_hist){
            btb_table[index].history = btb_table[index].history << 1;
            if(taken){
                btb_table[index].history++;
            }
            btb_table[index].history =  btb_table[index].history % result_table_size;
        }else{
            global_history = global_history << 1;
            if(taken){
                global_history++;
            }
            global_history = global_history % result_table_size;
        }

    }
    // If there is a relevant entry in table, we update.
    else{

        int current_hist;
        if(!is_global_hist){
            current_hist = btb_table[index].history;
        }else{
            current_hist = global_history;
        }

        int access_index;
        if(is_global_table && share_status == USING_SHARE_LSB){
            unsigned int calc_pc_lsb = pc >> 2;
            access_index =  (calc_pc_lsb ^ current_hist) % result_table_size;
        }else if(is_global_table && share_status == USING_SHARE_MID){
            unsigned int calc_pc_mid = pc >> 16;
            access_index =  (calc_pc_mid ^ current_hist) % result_table_size;
        }else{
            access_index = current_hist % result_table_size;
        }

        unsigned int* current_result_table;
        if(is_global_table){
            current_result_table = global_result_table;
        }else{
            current_result_table = btb_table[index].result_table;
        }

        if(taken && (current_result_table[access_index] != STRONGLY_TAKEN)){
            current_result_table[access_index]++;
        }else if(!taken && (current_result_table[access_index] != STRONGLY_NOT_TAKEN)){
            current_result_table[access_index]--;
        }

        if(!is_global_hist){
            btb_table[index].history = btb_table[index].history << 1;
            if(taken){
                btb_table[index].history++;
            }
            btb_table[index].history =  btb_table[index].history % result_table_size;
        }else{
            global_history = global_history << 1;
            if(taken){
                global_history++;
            }
            global_history = global_history % result_table_size;
        }
    }

}

void BP_GetStats(SIM_stats *curStats){
    curStats -> br_num = num_of_updates;
    curStats -> flush_num = num_of_flushes;
    curStats -> size = theoretical_memory_size;

    free(btb_table);
}

