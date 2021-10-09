#!/bin/bash
#Usage ./run_ijcai18_slurm.sh 

nsims=100
alpha=0.1
reps=25
verbosity=-1
min_time=-1
max_time=-1
other_flags=""

problems=( "--track=../data/tracks/known/square-4-error.track --perror=0.25 --pslip=0.50" \
           "--track=../data/tracks/known/ring-5-error.track --perror=0.25 --pslip=0.50" \
           "--sailing-size=40 --sailing-goal=39" \
           "--sailing-size=40 --sailing-goal=20")

problems_str=(square4 ring5 sailing-corner sailing-middle)
                
rhos=(0.0625 0.03125)
distfuns=(depth traj)          
labelfuns=(linear exp logistic)
for ((ip = 0; ip < ${#problems[@]}; ip++)); do
  problem=${problems[$ip]}
  problem_str=${problems_str[$ip]}
  
  # HDP(0)
  sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"hdp_0".txt \
    run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
    "hdp --i=0"
    
  # HDP(0,0)
  sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"hdp_00".txt \
    run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
    "hdp --j=0 --i=0"
    
  # Trajectory-based SSiPP
  for rho in ${rhos[@]}; do
    sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"ssipp_$rho".txt \
      run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
      "ssipp --rho=$rho"
  done
  
  for horizon in `seq 1 4`; do
    # Depth-based SSiPP(1-4)
    sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"ssipp_$horizon".txt \
      run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
      "ssipp --horizon=$horizon"
      
    # FLARES(1-4)
    sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"flares_$horizon".txt \
      run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
      "soft-flares --labelf=step --dist=depth --horizon=$horizon --alpha=0"
  done
    
  # Soft-FLARES(0)
  sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"soft-flares_0".txt \
    ./run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
    "soft-flares --labelf=linear --dist=depth --horizon=0 --alpha=$alpha"
  
  # Soft-FLARES(1)
  # In this case the label function is irrelevant because
  # all of them result in the same set of labeling probabilities.
  # However, the distance function can potentially matter.   
  for distf in ${distfuns[@]}; do
    sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"soft-flares_1_$distf".txt \
      run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
      "soft-flares --labelf=linear --dist=$distf --horizon=1 --alpha=$alpha"
  done
  
  # Soft-FLARES([2,4])
  for labelf in ${labelfuns[@]}; do
    for distf in ${distfuns[@]}; do
      for horizon in `seq 2 4`; do
        sbatch --output=/home/lpineda/results_ijcai18/${problem_str}_"soft-flares_${horizon}_${distf}_${labelf}".txt \
          run_testsolver.sh "$problem" $nsims $reps $verbosity $min_time $max_time "$other_flags" \
          "soft-flares --labelf=$labelf --dist=$distf --horizon=$horizon --alpha=$alpha"
      done
    done
  done
done

