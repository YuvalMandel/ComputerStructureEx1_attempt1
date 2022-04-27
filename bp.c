/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bp_api.h"

#define STRONGLY_NOT_TAKEN 0
#define WEAKLY_NOT_TAKEN 1
#define WEAKLY_TAKEN 2
#define STRONGLY_TAKEN 3

struct btb_entry{
    int tag;
    uint32_t target;
    int history;
    unsigned* result_table;
};

struct btb_entry* btb_table;

unsigned history_max;
unsigned tag_remove_value;
unsigned tag_find_divide_value;
int is_shared;

unsigned* global_result_table;
int global_history = 0;
bool is_global_hist;
bool is_global_table;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){

    btb_table = (struct btb_entry*)malloc(btbSize * sizeof(struct btb_entry));
    int result_table_size = pow(2,historySize);
    history_max = (unsigned)result_table_size;
    tag_remove_value = pow(2,32 - tagSize);
    tag_find_divide_value = pow(2,tagSize);
    is_shared = Shared;
    is_global_hist = isGlobalHist;
    is_global_table = isGlobalTable;

    // Local history, local table (shared irrelevt).
    if(!isGlobalHist && !isGlobalTable) {
        for (int i = 0; i < btbSize; ++i) {
            btb_table[i].history = 0;
            btb_table[i].result_table = (unsigned *) malloc(result_table_size * sizeof(unsigned));
            for (int j = 0; j < result_table_size; ++j) {
                btb_table[i].result_table[j] = fsmState;
            }
        }
    }else if(!isGlobalHist && isGlobalTable){
        for (int i = 0; i < btbSize; ++i) {
            btb_table[i].result_table = (unsigned *) malloc(result_table_size * sizeof(unsigned));
        }
        for (int j = 0; j < result_table_size; ++j) {
            global_result_table[j] = fsmState;
        }
    }else if(isGlobalHist && isGlobalTable){
        for (int j = 0; j < result_table_size; ++j) {
            global_result_table[j] = fsmState;
        }
    }else if(isGlobalHist && !isGlobalTable){
        for (int i = 0; i < btbSize; ++i) {
            btb_table[i].result_table = (unsigned *) malloc(result_table_size * sizeof(unsigned));
            for (int j = 0; j < result_table_size; ++j) {
                btb_table[i].result_table[j] = fsmState;
            }
        }
    }

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

    int index = pc % tag_remove_value;
    int tag = pc/tag_find_divide_value;

    if(btb_table[index].tag == tag){

        // Determine history.
        int current_hist;
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

        int access_index;
        if(is_global_table && is_shared){
            access_index = index ^ current_hist;
        }else{
            access_index = index;
        }

        unsigned result = current_result_table[access_index];

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
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

