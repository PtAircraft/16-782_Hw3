/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <vector>
#include <unordered_set>
#include <iostream>
#include <chrono>

/* Input Arguments */
#define	BLOCKSV_IN      prhs[0]
#define	TRIANGLESV_IN      prhs[1]
#define	TABLEINDEX_IN      prhs[2]
#define	ONVSTART_IN      prhs[3]
#define	CLEARVSTART_IN      prhs[4]
#define	ONVGOAL_IN      prhs[5]
#define	CLEARVGOAL_IN      prhs[6]
#define	MOVEACTIONINDEX_IN      prhs[7]
#define	MOVETOTABLEACTIONINDEX_IN      prhs[8]

using namespace std;
/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

const double cost = 1;
struct state {
	std::vector<std::vector<int>> onV_;
	std::vector<int> clearV_;
	std::vector<int> action_;

	state* parent_;
	double h_;
	double g_;
	double f_;
	state () {
		parent_ = nullptr;
		h_ = 0;
		g_ = 0;
		f_ = g_ + h_;

	}
};
// compare function for identify state
int compare_index(const state* state1, const state* state2) {
	if (state1->onV_.size() != state2->onV_.size() 
		|| state1->clearV_.size() != state2->clearV_.size()) {
		return 0;
	}
	int found;
	// check onV_
	for (int i = 0; i < state1->onV_.size(); i++) {
		found = 0;
		for (int j = 0; j < state2->onV_.size(); j++) {
			if (state1->onV_[i][0] == state2->onV_[j][0]
				&& state1->onV_[i][1] == state2->onV_[j][1]) {
				found = 1;
				continue;
			}
		}
		if (found == 0) {
			return 0;
		}
	}
	// check clear
	for (int i = 0; i < state1->clearV_.size(); i++) {
		found = 0;
		for (int j = 0; j < state2->clearV_.size(); j++) {
			if (state1->clearV_[i] == state2->clearV_[j]) {
				found = 1;
				continue;
			}
		}
		if (found == 0) {
			return 0;
		}
	}
	return 1;
}
// compare function for state g
struct compare_g {
	bool operator() (const state* state1, const state* state2) {
		return state1->f_ > state2->f_;
	}
};

//blocksV - is an array of block indices (that is, if b is in blocks, then b
//is a block
//numofblocks - length of blocksV
//trianglesV - is an array of triangle indices
//numoftriangles - lengt of trianglesV
//TableIndex - index that corresponds to the table
//onV_start - a vector of On(x,y) statements that are true at start (each row is a statement, so
//OnV_start[0] says that item OnV_start[0][0] is on top of OnV_start[0][1]). Note: the 2nd dimension of OnV_start has 2 elements 
//onV_start_length - number of statements in OnV_start (that is, the size of the 1st dimension in OnV_start)
//clearV_start - is an array of items that are clear at start state (note Table is always clear
//by default)
//numofclear_start - length of clearV_start
//onV_goal - a vector of On(x,y) statements that are true at goal (same format as onV_start)
//onV_goal_length - number of statements in OnV_goal (that is, the size of the 1st dimension in OnV_goal)
//clearV_goal - is an array of items that are clear at goal state 
//numofclear_goal - length of clearV_goal
//moveActionIndex - index of the move(x,y,z) action that moves x from y to z
//(note that y could be a table but z should NOT be a table)
//moveToTableActionIndex - index of the moveToTable(x,y,z) action that moves x
//from y to z, where z is ALWAYS an index of the table
//plan - an array of action with their parameters. plan(i,:) is ith action.
//plan[i][0] - index of the action, where plan[i][1] - first argument, plan[i][2] - second argument, plan[i][3] - third argument 
static void planner(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
            int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
            int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal, 
            int moveActionIndex, int moveToTableActionIndex, int*** plan, int* planlength) 
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // initialize start state;
    state* start_state = new state();
    for (int i = 0; i < onV_start_length; i++) {
    	std::vector<int> tmp_on;
    	tmp_on.push_back(onV_start[i][0]);
    	tmp_on.push_back(onV_start[i][1]);
    	start_state->onV_.push_back(tmp_on);
    }
    for (int i = 0; i < numofclear_start; i++) {
    	start_state->clearV_.push_back(clearV_start[i]);
    }

    // Put triangles in unordered set to improve speed
    std::unordered_set<int> trianglesV_us;
    for (int i = 0; i < numoftriangles; i ++) {
    	trianglesV_us.insert(trianglesV[i]);
    }
    // initialize open list
    std::vector<state*> open;
    open.push_back(start_state);
    make_heap(open.begin(), open.end(), compare_g());
    // initialize closed list
    std::vector<state*> closed;
    // flag for target obj
    int flag_tri = 0;
    // flag for target location
    int flag_ontable = 1;
    int found_in_closed = 0;
    // A* search
    static state* s0;
    int n = 0;
    while(!open.empty()) {
    	n ++;
    	// if (n > 1000000) break;
    	s0 = open.front();
        pop_heap(open.begin(), open.end(), compare_g());
        open.pop_back();
        closed.push_back(s0);
        // check if goal is expended
        int clear_count = 0;
        for (int i = 0; i < numofclear_goal; i++) {
        	for (int j = 0; j < s0->clearV_.size(); j++) {
        		if (s0->clearV_[j] == clearV_goal[i]) {
        			clear_count ++;
        		}
        	}
        }

        if (clear_count == numofclear_goal){
        	int on_count = 0;
        	for (int i = 0; i < onV_goal_length; i ++) {
        		for (int j = 0; j < s0->onV_.size(); j++) {
        			if (s0->onV_[j][0] == onV_goal[i][0] 
        				&& s0->onV_[j][1] == onV_goal[i][1]) {
        				on_count ++;
        			}
        		}
        	}
        	if (on_count == onV_goal_length) {
        		std::cout << "Solutoin Found!" << endl;
        		break;
        	}
        }
        // find successor
        // search through the clear obj, select the obj to move
        for (int i = 0; i < s0->clearV_.size(); i++) {
        	// cout << "check 5" << endl;
        	int obj_selected = s0->clearV_[i];
        	// check where the selected obj is;
        	// where the block that obj is on;
        	int under_obj;
        	flag_ontable = 1;
    		// check if it's on the first element of on-vector
        	for (int l = 0; l < s0->onV_.size(); l++) {
        		if (obj_selected == s0->onV_[l][0]) {
        			// not on table
        			flag_ontable = 0;
        			under_obj = s0->onV_[l][1];
        			break;
        		}
        	}
        	// if selected obj is not on table, move to table and create new state
        	if (!flag_ontable) {
	        	// build successor state
				state* successor = new state();
				successor->g_ = s0->g_ + cost;
				successor->clearV_ = s0->clearV_;
				successor->onV_ = s0->onV_;
				successor->parent_ = s0;
				// action to get to the successor
				std::vector<int> action;
				action.push_back(moveToTableActionIndex);
				action.push_back(obj_selected);
				action.push_back(under_obj);
				action.push_back(TableIndex);
				successor->action_ = action;
				// find the state after apply action(aka state for successor)
				// add under obj to clear
				successor->clearV_.push_back(under_obj);
				// delete on(selected_obj, under_obj)
				for (int l = 0; l < successor->onV_.size(); l++) {
					if (obj_selected == successor->onV_[l][0]) {
						successor->onV_.erase(successor->onV_.begin() + l);
						break;
					}
				}
				// compute h and update f when states are established
				int count = numofclear_goal + onV_goal_length;
				for (int i = 0; i < successor->clearV_.size(); i++) {
					for (int j = 0; j < numofclear_goal; j++) {
						if (successor->clearV_[i] == clearV_goal[j]){
							count --;
							break;
						} 
					}
				}
				for (int i = 0; i < successor->onV_.size(); i++) {
					for (int j = 0; j < onV_goal_length; j ++) {
						if (successor->onV_[i][0] == onV_goal[j][0] 
							&& successor->onV_[i][1] == onV_goal[j][1]) {
						count --;
						break;
						}
					}
				}
				successor->h_ = count;
				successor->f_ = successor->h_ + successor->g_;
				// after all successor's states are updated, check if it's in closed
				found_in_closed = 0;
				for (int i = 0; i < closed.size(); i ++) {
					if (compare_index(successor, closed[i])) {
						found_in_closed = 1;
						break;
					}
				}
				// if it's in closed, skip to next successor
				bool found_in_open = false;
				if (found_in_closed == 1) {
					// continue;
				} else {
					// if it's not in clsoed, check if it's in open
					for (int i = 0; i < open.size(); i++) {
						// if in open
						if (compare_index(successor, open[i])) {
							if (successor->g_ + cost < open[i]->g_) {
								open[i]->g_ = successor->g_ + cost;
								open[i]->f_ = open[i]->g_ + open[i]->h_;
								open[i]->parent_ = successor->parent_;
								make_heap(open.begin(), open.end(), compare_g());
								found_in_open = true;
								break;
							}
						}
					}
					// if it's not in open, insert successor into open
					if (!found_in_open) {
						open.push_back(successor);
						push_heap(open.begin(), open.end(), compare_g());
					}
				}
        		
        	}

			// *******************************************************************//
        	// search through the rest clear objs, 
        	// select the un-triangle target obj to place moved obj
        	for (int j = 0; j < s0->clearV_.size(); j++) {
        		// if it's not the same obj
        		if (i != j) {
        			int obj_target = s0->clearV_[j];
        			// check if the target obj is triangle
    				flag_tri = 0;
    				if (trianglesV_us.find(obj_target) != trianglesV_us.end()) {
    					flag_tri = 1;
    				}

        			// if target obj is triangle
        			if (flag_tri == 1) {
        				// and selected obj is on table
        				if (flag_ontable == 1) {
        					// do nothing and break to find next successor
        					continue;
        				}
        			} else { 
	    				// create successor
	    				state* successor = new state();
						successor->g_ = s0->g_ + cost;
						// h to be found
						// successor->f_ = successor->h_ + successor->g_;
						successor->clearV_ = s0->clearV_;
						successor->onV_ = s0->onV_;
						successor->parent_ = s0;
        				// if target obj is not triangle, 
	    				// move the selected obj to target obj from table
	    				if (flag_ontable == 1) {
		    				// std::vector<int> action;
		    				successor->action_.push_back(moveActionIndex);
		    				successor->action_.push_back(obj_selected);
		    				successor->action_.push_back(TableIndex);
		    				successor->action_.push_back(obj_target);
							// update state
							// delete obj_target from clearV in successor
							for (int i = 0; i < successor->clearV_.size(); i++) {
								if (obj_target == successor->clearV_[i]) {
									successor->clearV_.erase(successor->clearV_.begin() + i);
									break;
								}
							}
							// add obj and obj_target into onV
							std::vector<int> tmp_on;
							tmp_on.push_back(obj_selected);
							tmp_on.push_back(obj_target);
							successor->onV_.push_back(tmp_on);
	    				} else {
	    					// from under_obj
	    					// std::vector<int> action;
		    				successor->action_.push_back(moveActionIndex);
		    				successor->action_.push_back(obj_selected);
		    				successor->action_.push_back(under_obj);
		    				successor->action_.push_back(obj_target);
		    				// add under_obj into clearV
		    				successor->clearV_.push_back(under_obj);
		    				// erase obj_target in clear
		    				for (int i = 0; i < successor->clearV_.size(); i++) {
		    					if (successor->clearV_[i] == obj_target) {
		    						successor->clearV_.erase(successor->clearV_.begin() + i);
		    						break;
		    					}
		    				}
		    				// add on obj_selected obj_target into onV
							std::vector<int> tmp_on;
							tmp_on.push_back(obj_selected);
							tmp_on.push_back(obj_target);
							successor->onV_.push_back(tmp_on);
							// erase obj_selected and under-obj in onV
							// delete on(selected_obj, under_obj)
							for (int l = 0; l < successor->onV_.size(); l++) {
								if (obj_selected == successor->onV_[l][0]) {
									successor->onV_.erase(successor->onV_.begin() + l);
									break;
								}
							}
	    				}
						// compute h and update f when states are established
						int count = numofclear_goal + onV_goal_length;
						for (int i = 0; i < successor->clearV_.size(); i++) {
							for (int j = 0; j < numofclear_goal; j++) {
								if (successor->clearV_[i] == clearV_goal[j]){
									count --;
									break;
								} 
							}
						}
						for (int i = 0; i < successor->onV_.size(); i++) {
							for (int j = 0; j < onV_goal_length; j ++) {
								if (successor->onV_[i][0] == onV_goal[j][0] && successor->onV_[i][1] == onV_goal[j][1]) {
								count --;
								break;
								}
							}
						}
						successor->h_ = count;
						successor->f_ = successor->h_ + successor->g_;
						// after all successor's states are updated, check if it's in closed
						found_in_closed = 0;
						// cout << "check 16" << endl;
						for (int i = 0; i < closed.size(); i ++) {
							if (compare_index(successor, closed[i])) {
								found_in_closed = 1;
								break;
							}
						}
						// if it's in closed, skip to next successor
						if (found_in_closed == 1) {
							// continue;
						} else {
							// if it's not in clsoed, check if it's in open
							bool found_in_open = false;
							for (int i = 0; i < open.size(); i++) {
								// if in open
								if (compare_index(successor, open[i])) {
									if (successor->g_ + cost < open[i]->g_) {
										open[i]->g_ = successor->g_ + cost;
										open[i]->f_ = open[i]->g_ + open[i]->h_;
										open[i]->parent_ = successor->parent_;
										make_heap(open.begin(), open.end(), compare_g());
										found_in_open = true;
										break;
									}
								}
							}
							// if it's not in open, insert successor into open
							if (!found_in_open) {
								open.push_back(successor);
								push_heap(open.begin(), open.end(), compare_g());
							}
						}
        			}
        		}
        	}
        }
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_in_search = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    printf("Search time:  %fs\n",time_in_search);

    // Back search for solution
    // start from goal state
    state* state = closed.back();
    int size = 0;
    while (state != nullptr) {
    	size++;
    	state = state->parent_;
    }
    size = size - 1;
    // pre-allocate memory
    *plan = (int**) malloc(size*sizeof(int*));
    state = closed.back();
    // cout << size << endl;
    // cout << "size: " << state->action_.size() << endl;
    for (int i = size; i > 0; i--) {
    	// cout << i << endl;
    	// cout << "check1" << endl;
        (*plan)[i- 1] = (int*) malloc(4*sizeof(int)); 
        // cout << state->action_[0] << endl;
        // cout << "check2" << endl;
        (*plan)[i- 1][0] = state->action_[0];
        // cout << "check3" << endl;
        (*plan)[i- 1][1] = state->action_[1];
        // cout << "check4" << endl;
        (*plan)[i- 1][2] = state->action_[2];
        // cout << "check5" << endl;
        (*plan)[i- 1][3] = state->action_[3];
        state = state->parent_;
    }
    *planlength = size;

    return;
}

//prhs contains input parameters (9): 
//1st is blocksV - is an array of block indices (that is, if b is in blocks, then b
//is a block
//2nd is trianglesV - is an array of triangle indices
//3rd is TableIndex - index that corresponds to the table
//4th is onV_start - a vector of On(x,y) statements that are true at start (each row is a statement, so
//OnV[0] says that item OnV[0][0] is on top of OnV[0][1]
//5th is clearV_start - is an array of items that are clear at start state (note Table is always clear
//by default)
//6th is onV_goal - a vector of On(x,y) statements that are true at goal 
//7th i clearV_goal - is an array of items that are clear at goal state 
//8th is moveActionIndex - index of the move(x,y,z) action that moves x from y to z
//(note that y could be a table but z should NOT be a table)
//9th is moveToTableActionIndex - index of the moveToTable(x,y,z) action that moves x
//from y to z, where z is ALWAYS an index of the table

//plhs should contain output parameters (1): 
//plan - an array of action with their parameters. plan(i,:) is ith action.
//plan[i][0] - index of the action, where plan[i][1] - first argument, plan[i][2] - second argument, plan[i][3] - third argument 
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 9) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Nine input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 

    int i = 0;

    /* get the blocks */
    double* blocksV_double = mxGetPr(BLOCKSV_IN);
    int numofblocks = (int) (MAX(mxGetM(BLOCKSV_IN), mxGetN(BLOCKSV_IN)));
    if(numofblocks < 2)
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumofBlocks",
                "At least two blocks are required.");         
    int *blocksV = (int*) malloc(sizeof(int)*numofblocks);
    for(i = 0; i < numofblocks; i++)
    {
        blocksV[i] = (int) blocksV_double[i];
        printf("block %d = %d\n", i, blocksV[i]);
    }
    
    /* get the triangles */
    double* trianglesV_double = mxGetPr(TRIANGLESV_IN);
    int numoftriangles = (int) (MAX(mxGetM(TRIANGLESV_IN), mxGetN(TRIANGLESV_IN)));
    int *trianglesV = (int*) malloc(sizeof(int)*numoftriangles);
    for(i = 0; i < numoftriangles; i++)
    {
        trianglesV[i] = (int) trianglesV_double[i];
        printf("triangle %d = %d\n", i, trianglesV[i]);
    }

    /*get the table index */
    int TableIndex = (int)(*mxGetPr(TABLEINDEX_IN));
    printf("TableIndex=%d\n", TableIndex);
    
    /*get the onV for start*/
    int onV_start_length = (int) mxGetM(ONVSTART_IN);
    int onV_start_cols = (int) mxGetN(ONVSTART_IN);
    double* onv_start_double = mxGetPr(ONVSTART_IN);
    if(onV_start_cols != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidonv_start",
                "each onv_start statement should have 3 parameters");         
    }
    int **onV_start = (int**) malloc(sizeof(int*)*onV_start_length);
    for(i = 0; i < onV_start_length; i++)
    {
        onV_start[i] = (int*)(malloc(sizeof(int)*2));
        onV_start[i][0] = (int)onv_start_double[0*onV_start_length + i];
        onV_start[i][1] = (int)onv_start_double[1*onV_start_length + i];
        printf("OnV at start %d: %d is on %d\n", i, onV_start[i][0], onV_start[i][1]);
    }
        
    /*get the clearV for start*/
    double* clearV_start_double = mxGetPr(CLEARVSTART_IN);
    int numofclear_start = (int) (MAX(mxGetM(CLEARVSTART_IN), mxGetN(CLEARVSTART_IN)));
    int *clearstartV = (int*) malloc(sizeof(int)*numofclear_start);
    for(i = 0; i < numofclear_start; i++)
    {
        clearstartV[i] = (int) clearV_start_double[i];
        printf("clear at start %d: %d is clear\n", i, clearstartV[i]);
    }

    
    /*get the onV for goal*/
    int onV_goal_length = (int) mxGetM(ONVGOAL_IN);
    int onV_goal_cols = (int) mxGetN(ONVGOAL_IN);
    double* onv_goal_double = mxGetPr(ONVGOAL_IN);
    if(onV_goal_cols != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidonv_goal",
                "each onv_goal statement should have 3 parameters");         
    }
    int **onV_goal = (int**) malloc(sizeof(int*)*onV_goal_length);
    for(i = 0; i < onV_goal_length; i++)
    {
        onV_goal[i] = (int*) malloc(sizeof(int)*2);
        onV_goal[i][0] = (int)onv_goal_double[0*onV_goal_length + i];
        onV_goal[i][1] = (int)onv_goal_double[1*onV_goal_length + i];
        printf("OnV at goal %d: %d is on %d\n", i, onV_goal[i][0], onV_goal[i][1]);
    }
    
    /*get the clearV for goal*/
    double* clearV_goal_double = mxGetPr(CLEARVGOAL_IN);
    int numofclear_goal = (int) (MAX(mxGetM(CLEARVGOAL_IN), mxGetN(CLEARVGOAL_IN)));
    int *cleargoalV = (int*) malloc(sizeof(int)*numofclear_goal);
    for(i = 0; i < numofclear_goal; i++)
    {
        cleargoalV[i] = (int) clearV_goal_double[i];
        printf("clear at goal %d: %d is clear\n", i, cleargoalV[i]);
    }
    
    /*get the moveAction index */
    int moveActionIndex = (int)(*mxGetPr(MOVEACTIONINDEX_IN));
    printf("moveActionIndex=%d\n", moveActionIndex);
           
    /*get the moveToTableAction index */
    int moveToTableActionIndex = (int)(*mxGetPr(MOVETOTABLEACTIONINDEX_IN));
    printf("moveToTableActionIndex=%d\n", moveToTableActionIndex);
    
    //call the planner
    int** plan = NULL;
    int planlength = 0;
    
    planner(blocksV, numofblocks, trianglesV, numoftriangles, TableIndex, 
            onV_start, onV_start_length, clearstartV, numofclear_start, onV_goal, onV_goal_length, cleargoalV, numofclear_goal, 
            moveActionIndex, moveToTableActionIndex, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
        
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)4, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < 4; j++)
            {
                plan_out[j*planlength + i] = (double)plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        *plan_out = 0;
    }
            
            
    return;
    
}





