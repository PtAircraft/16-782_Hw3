	
    //this is where you insert your planner
    
    allocate memory and set the return plan
    for now this is an arbitrary sequence of actions (LIKELY INVALID)
    int numofsteps = 5;    
    *plan = (int**) malloc(numofsteps*sizeof(int*));
    int i;
    for (i = 0; i < numofsteps; i++){
        (*plan)[i] = (int*) malloc(4*sizeof(int)); 
        
        //just call move actions for even steps and movetotable actions for odd steps
        if(i%2 == 0)
        {
            //note this could be an invalid action since we are not checking if blocksV is clear and if it is on a table indeed
            (*plan)[i][0] = moveActionIndex;
            (*plan)[i][1] = blocksV[0];
            (*plan)[i][2] = TableIndex; 
            (*plan)[i][3] = blocksV[1];
            printf("%d %d %d %d\n", (*plan)[i][0], (*plan)[i][1], (*plan)[i][2], (*plan)[i][3]);
        }
        else
        {
            //note this could be an invalid action since we are not checking if blocksV is clear and if it is on a table indeed
            (*plan)[i][0] = moveToTableActionIndex;
            (*plan)[i][1] = blocksV[0];
            (*plan)[i][2] = blocksV[1]; 
            (*plan)[i][3] = TableIndex;                         
            printf("%d %d %d %d\n", (*plan)[i][0], (*plan)[i][1], (*plan)[i][2], (*plan)[i][3]);
        }
    }    
    *planlength = numofsteps;
    
    return;
    