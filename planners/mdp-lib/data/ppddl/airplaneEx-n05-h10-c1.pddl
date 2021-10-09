(define (domain airplaneEx)
  (:requirements :typing :strips :equality :probabilistic-effects :rewards)
  (:types
    agent
    location
    trajectory
    sensor
    issue
    step
  )
  (:constants
    ;The success issue is not too useful since not listing it with the closed-
    ;  world assumption still means there is no fault, but may be useful if a 
    ;  repair action is ever added to set 'success'
    success - issue
    fault_exists - issue
    joint_fault_exists - issue
    charge_station - location
    left_wing about_left_wing - location
    ;Need adjacent locations as no-go zones, but may be reached by random drift
    right_wing about_right_wing - location
    ;The other locations are not used for now
    ;left_horizontal_stabilizer about_left_horizontal_stabilizer - location
    ;right_right_right_engine about_right_right_right_engine - location
    ;left_right_right_engine about_left_right_right_engine - location
    ;right_horizontal_stabilizer about_right_horizontal_stabilizer - location
  )
  (:predicates
    (available ?a - agent)
    (valid ?a - agent ?t - trajectory ?initLoc - location ?finalLoc - location)
    (at ?a - agent ?loc - location)
    (status ?loc - location ?i - issue)
    (pair_status ?loc1 ?loc2 - location ?i - issue)
    (fault_location_detected ?loc - location)
    (hasSensor ?a - agent ?s - sensor)
    (battery_sufficient ?a - agent)
    (handshaken ?a1 ?a2 - agent)
    (informed ?loc - location)
    (consecutive ?s1 ?s2 - step)
    (current ?s - step)
    (is_adjacent ?loc - location)
    (inspecting ?a - agent)
    (initialized)
    (terminated)
    (confirmed ?loc - location)
  )
  (:action move
    :parameters (?a - agent ?t - trajectory ?initLoc ?finalLoc - location ?s1 ?s2 - step)
    :precondition (and
      (available ?a)
      (at ?a ?initLoc)
      (valid ?a ?t ?initLoc ?finalLoc)
      (battery_sufficient ?a)
      (current ?s1)
      (consecutive ?s1 ?s2)
      (not (is_adjacent ?finalLoc))
    )
    :effect (and
      (not (current ?s1))
      (current ?s2)
      ;Move to the new location, but may be slightly off due to error in odometry/position sensor/etc.
      (not (at ?a ?initLoc))
      ;(when (= ?finalLoc left_horizontal_stabilizer)
      ;  (probabilistic
      ;	   0.9 (at ?a left_horizontal_stabilizer)
      ;	   0.1 (at ?a about_left_horizontal_stabilizer)
      ;  )
      ;)
      ;(when (= ?finalLoc right_right_right_engine)
      ;  (probabilistic
      ;    0.9 (at ?a right_right_right_engine)
      ;	   0.1 (at ?a about_right_right_right_engine)
      ;	 )
      ;)
      (when (= ?finalLoc left_wing)
        (probabilistic
	  0.95 (at ?a left_wing)
	  0.05 (at ?a about_left_wing)
	)
      )
      (when (= ?finalLoc right_wing)
        (probabilistic
	  0.95 (at ?a right_wing)
	  0.05 (at ?a about_right_wing)
	)
      )
      ;(when (= ?finalLoc left_right_right_engine)
      ;  (probabilistic
      ;	   0.9 (at ?a left_right_right_engine)
      ;	   0.1 (at ?a about_left_right_right_engine)
      ;	 )
      ;)
      ;(when (= ?finalLoc right_horizontal_stabilizer)
      ;  (probabilistic
      ;	   0.9 (at ?a right_horizontal_stabilizer)
      ;	   0.1 (at ?a about_right_horizontal_stabilizer)
      ;	 )
      ;)
      (when (= ?finalLoc charge_station) (at ?a charge_station))
    )
  )
  (:action inspect_part1
    :parameters (?a - agent ?s - sensor ?t - trajectory ?loc - location ?s1 ?s2 - step)
    :precondition (and
      (not (inspecting ?a))
      (available ?a)
      (hasSensor ?a ?s)
      (at ?a ?loc)
      (valid ?a ?t ?loc ?loc)
      (battery_sufficient ?a)
      (current ?s1)
      (consecutive ?s1 ?s2)
      (not (is_adjacent ?loc))
    )
    :effect (and
      ;Begin inspecting, and temporarily not available
      (inspecting ?a)
      (not (available ?a))
      (not (current ?s1))
      (current ?s2)
    )
  )
  (:action inspect_part2
    :parameters (?a - agent ?s - sensor ?t - trajectory ?loc - location ?s1 ?s2 - step)
    :precondition (and
      (inspecting ?a)
      (not (available ?a))
      (hasSensor ?a ?s)
      (at ?a ?loc)
      (valid ?a ?t ?loc ?loc)
      (battery_sufficient ?a)
      (current ?s1)
      (consecutive ?s1 ?s2)
      (not (is_adjacent ?loc))
    )
    :effect (and
      ;Done inspecting, and can become available again
      (not (inspecting ?a))
      (available ?a)
      (not (current ?s1))
      (current ?s2)
      ;Decent chance of identifying a fault - no false negatives, just false positives
      (when (status ?loc fault_exists) (probabilistic 0.95 (fault_location_detected ?loc)))
      ;Joint faults are much more difficult for one agent to detect
      (when (status ?loc joint_fault_exists) (probabilistic 0.05 (fault_location_detected ?loc)))
      ;Potential drift to consider during the inspection
      (probabilistic 0.1 (and
      (when (= ?loc left_wing)
        (probabilistic
	  ;0.9 (at ?a left_wing)
	  0.05 (and (not (at ?a ?loc)) (at ?a about_left_wing))
	)
      )
      (when (= ?loc right_wing)
        (probabilistic
	  ;0.9 (at ?a right_wing)
	  0.05 (and (not (at ?a ?loc)) (at ?a about_right_wing))
	)
      )
      ))
    )
  )

  (:action recharge
    :parameters (?a - agent ?s1 ?s2 - step)
    :precondition (and
      (current ?s1)
      (consecutive ?s1 ?s2)
    )
    :effect (and
      (at ?a charge_station)
      (battery_sufficient ?a)
      (not (current ?s1))
      (current ?s2)
    )
  )

  (:action alert
    :parameters (?a - agent ?loc - location ?s1 ?s2 - step)
    :precondition (and
      (available ?a)
      (fault_location_detected ?loc)
      (current ?s1)
      (consecutive ?s1 ?s2)
    )
    :effect (and
      (not (current ?s1))
      (current ?s2)
      (informed ?loc)
    )
  )
  (:action handshake
    :parameters (?a1 ?a2 - agent ?s1 ?s2 - step)
    :precondition (and
      (available ?a1)
      (not (= ?a1 ?a2))
      (current ?s1)
      (consecutive ?s1 ?s2)
    )
    :effect (and
      (not (current ?s1))
      (current ?s2)
      ;If already handshaken, then no change to make
      (when (not (or (handshaken ?a1 ?a2) (handshaken ?a2 ?a1)))
        (probabilistic 0.7 (handshaken ?a1 ?a2))
      )
    )
  )

  (:action joint_inspect
    :parameters (?a1 ?a2 - agent ?sense1 ?sense2 - sensor ?t1 ?t2 - trajectory ?loc1 ?loc2 - location ?s1 ?s2 - step)
    :precondition (and
      (current ?s1)
      (consecutive ?s1 ?s2)
      (not (= ?a1 ?a2))
      (available ?a1)
      (hasSensor ?a1 ?sense1)
      (at ?a1 ?loc1)
      (valid ?a1 ?t1 ?loc1 ?loc1)
      (battery_sufficient ?a1)
      (available ?a2)
      (hasSensor ?a2 ?sense2)
      (at ?a2 ?loc2)
      (valid ?a2 ?t2 ?loc2 ?loc2)
      (battery_sufficient ?a2)
      (not (is_adjacent ?loc1))
      (not (is_adjacent ?loc2))
      ;Must already agree to work together
      (or (handshaken ?a1 ?a2) (handshaken ?a2 ?a1))
    )
    :effect (and
      (not (current ?s1))
      (current ?s2)
      ;Joint Inspection increases chance of finding a standard fault
      ;  Equation is P(?a1 or ?a2 finds fault) = [P(?a1 finds) + P(?a2 finds)] - [P(?a1 finds) * P(?a2 finds)]
      (when (and (= loc1 loc2) (status ?loc1 fault_exists)) (probabilistic 0.99 (fault_location_detected ?loc1)))
      ;Joint Inspection increases chances of finding the less noticable faults
      (when (and (= loc1 loc2) (status ?loc1 joint_fault_exists)) (probabilistic 0.9 (fault_location_detected ?loc1)))
      ;Joint Inspection only can find the pairwise faults (between two locations)
      (when (or (pair_status ?loc1 ?loc2 joint_fault_detected) (pair_status ?loc2 ?loc1 joint_fault_detected)) (probabilistic 0.9 (and (joint_fault_detected ?loc1 ?loc2) (joint_fault_detected ?loc2 ?loc1))))
      ;No longer need to coordinate unless agree again in the future
      (not (handshaken ?a1 ?a2))
      (not (handshaken ?a2 ?a1))
      ;Potential drift to consider during the inspection for ?a1
      (probabilistic 0.1 (and
      (not (at ?a1 ?loc1))
      ;(when (= ?loc1 left_horizontal_stabilizer)
      ;  (probabilistic
      ;	   0.9 (at ?a1 left_horizontal_stabilizer)
      ;	   0.1 (at ?a1 about_left_horizontal_stabilizer)
      ;  )
      ;)
      ;(when (= ?loc1 right_right_right_engine)
      ;  (probabilistic
      ;    0.9 (at ?a1 right_right_right_engine)
      ;	   0.1 (at ?a1 about_right_right_right_engine)
      ;	 )
      ;)
      (when (= ?loc1 left_wing)
        (probabilistic
	  0.9 (at ?a1 left_wing)
	  0.1 (at ?a1 about_left_wing)
	)
      )
      (when (= ?loc1 right_wing)
        (probabilistic
	  0.9 (at ?a1 right_wing)
	  0.1 (at ?a1 about_right_wing)
	)
      )
      ;(when (= ?loc1 left_right_right_engine)
      ;  (probabilistic
      ;	   0.9 (at ?a1 left_right_right_engine)
      ;	   0.1 (at ?a1 about_left_right_right_engine)
      ;	 )
      ;)
      ;(when (= ?loc1 right_horizontal_stabilizer)
      ;  (probabilistic
      ;	   0.9 (at ?a1 right_horizontal_stabilizer)
      ;	   0.1 (at ?a1 about_right_horizontal_stabilizer)
      ;	 )
      ;)
      (when (= ?loc1 charge_station) (at ?a1 charge_station))
      ))
      ;Potential drift to consider during the inspection for ?a2
      (probabilistic 0.1 (and
      (not (at ?a2 ?loc2))
      ;(when (= ?loc2 left_horizontal_stabilizer)
      ;  (probabilistic
      ;	   0.9 (at ?a2 left_horizontal_stabilizer)
      ;	   0.1 (at ?a2 about_left_horizontal_stabilizer)
      ;  )
      ;)
      ;(when (= ?loc2 right_right_right_engine)
      ;  (probabilistic
      ;    0.9 (at ?a2 right_right_right_engine)
      ;	   0.1 (at ?a2 about_right_right_right_engine)
      ;	 )
      ;)
      (when (= ?loc2 left_wing)
        (probabilistic
	  0.9 (at ?a2 left_wing)
	  0.1 (at ?a2 about_left_wing)
	)
      )
      (when (= ?loc2 right_wing)
        (probabilistic
	  0.9 (at ?a2 right_wing)
	  0.1 (at ?a2 about_right_wing)
	)
      )
      ;(when (= ?loc2 left_right_right_engine)
      ;  (probabilistic
      ;	   0.9 (at ?a2 left_right_right_engine)
      ;	   0.1 (at ?a2 about_left_right_right_engine)
      ;	 )
      ;)
      ;(when (= ?loc2 right_horizontal_stabilizer)
      ;  (probabilistic
      ;	   0.9 (at ?a2 right_horizontal_stabilizer)
      ;	   0.1 (at ?a2 about_right_horizontal_stabilizer)
      ;	 )
      ;)
      (when (= ?loc2 charge_station) (at ?a2 charge_station))
      ))
    )
  )
  ;The craziest hack ever for the sake of replanning with mini-gpt
  ;  Initialize all facts here rather than in the initial state
  (:action setup_world
    ;DEPRECATED parameters no longer needed for setup_world
    ;:parameters (?a - agent)
    :precondition (not (initialized))
    :effect (and
      (initialized)
      ;New things to add for the second quadrotor
      ;(at quadrotor2 charge_station)
      ;(available quadrotor2)
      ;(hasSensor quadrotor2 camera)
      ;(valid quadrotor2 defaultTraj charge_station left_wing)
      ;(valid quadrotor2 defaultTraj left_wing charge_station)
      ;Literally copy-and-paste the entire initial state
      (consecutive s0 s1)
          ;(consecutive s1 s2)
          ;(consecutive s2 s3)
          ;(consecutive s3 s4)
          ;(consecutive s4 s5)
          ;(consecutive s5 s6)
          ;(consecutive s6 s7)
          ;(consecutive s7 s8)
          ;(consecutive s8 s9)
          ;(consecutive s9 s10)
          ;(consecutive s10 s0)

          (valid quadrotor1 defaultTraj left_wing left_wing)
          (valid quadrotor1 defaultTraj right_wing left_wing)
          (valid quadrotor1 defaultTraj charge_station left_wing)
          (valid quadrotor1 defaultTraj left_wing right_wing)
          (valid quadrotor1 defaultTraj right_wing right_wing)
          (valid quadrotor1 defaultTraj charge_station right_wing)
          (valid quadrotor1 defaultTraj left_wing charge_station)
          (valid quadrotor1 defaultTraj right_wing charge_station)
          (valid quadrotor1 defaultTraj charge_station charge_station)

          (is_adjacent about_left_wing)
          (is_adjacent about_right_wing)
          (valid quadrotor1 defaultTraj about_left_wing left_wing)
          (valid quadrotor1 defaultTraj about_left_wing charge_station)
          (valid quadrotor1 defaultTraj about_right_wing right_wing)
          (valid quadrotor1 defaultTraj about_right_wing charge_station)

          (status right_wing fault_exists)
          (status left_wing fault_exists)
          (status charge_station success)

          (hasSensor quadrotor1 camera)
    (at quadrotor1 charge_station)
    (available quadrotor1)
    (battery_sufficient quadrotor1)
    ;Assume time 0 and each increments one-by-one
    (current s0)
    )
  )
  (:action done
    ;DEPRECATED parameters no longer needed for done
    ;:parameters (?a - agent)
    :precondition (and
      ;NOTE: COPY THE GOAL CONDITIONS LIST HERE! - UPDATE FOR EACH NEW PROBLEM
      ;(or (status left_horizontal_stabilizer success) (informed left_horizontal_stabilizer))
      ;(or (status right_right_right_engine success) (informed right_right_right_engine))
      (or (status left_wing success) (confirmed left_wing))
      (or (status right_wing success) (confirmed right_wing))(status right_wing fault_exists)
      ;(or (status left_right_right_engine success) (informed left_right_right_engine))
      ;(or (status right_horizontal_stabilizer success) (informed right_horizontal_stabilizer))
      (or (status charge_station success) (confirmed charge_station))
      ;(at quadrotor1 left_wing)
    )
    :effect (terminated)
  )
  (:action human_confirm
    :parameters ( ?loc - location ?s1 - step ?s2 - step )
    :precondition   (and
      (informed ?loc)
      (current ?s1)
      (consecutive ?s1 ?s2)
      (not (confirmed ?loc))
    )
    :effect   (and
      (confirmed ?loc)
      (not   (current ?s1)
      )
      (current ?s2)
    )
  )
)
(define (problem p01)
  (:domain airplaneEx)
  (:objects
    quadrotor1 - agent
    ;quadrotor2 - agent
    camera - sensor
    defaultTraj - trajectory
    s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 - step
    ;Everything else is a constant in the domain definition
  )
  (:init
    (consecutive s0 s1)
    (consecutive s1 s2)

    (consecutive s2 s3)
    (consecutive s3 s4)
    (consecutive s4 s5)
    (consecutive s5 s6)
    (consecutive s6 s7)
    (consecutive s7 s8)
    (consecutive s8 s9)
    ;(consecutive s9 s10)
    ;(consecutive s10 s0)

  ;  (valid quadrotor1 defaultTraj left_wing left_wing)
  ;  (valid quadrotor1 defaultTraj right_wing left_wing)
  ;  (valid quadrotor1 defaultTraj charge_station left_wing)
  ;  (valid quadrotor1 defaultTraj left_wing right_wing)
  ;  (valid quadrotor1 defaultTraj right_wing right_wing)
  ;  (valid quadrotor1 defaultTraj charge_station right_wing)
  ;  (valid quadrotor1 defaultTraj left_wing charge_station)
  ;  (valid quadrotor1 defaultTraj right_wing charge_station)
  ;  (valid quadrotor1 defaultTraj charge_station charge_station)

  ;  (is_adjacent about_left_wing)
  ;  (is_adjacent about_right_wing)
  ;  (valid quadrotor1 defaultTraj about_left_wing left_wing)
  ;  (valid quadrotor1 defaultTraj about_left_wing charge_station)
  ;  (valid quadrotor1 defaultTraj about_right_wing right_wing)
  ;  (valid quadrotor1 defaultTraj about_right_wing charge_station)

    ;Assume we know the fault regions for now
  ;  (status left_wing fault_exists)
  ;  (status right_wing fault_exists)
  ;  ;(status left_wing success)
  ;  (status charge_station success)

  ;  (hasSensor quadrotor1 camera)
)
 ;  (:goal (and
 ;   ;(or (status left_wing success) (confirmed left_wing))
 ;   (or (status right_wing success) (confirmed right_wing))
 ;   (or (status charge_station success) (confirmed charge_station))
 ;   ;(at quadrotor1 left_wing)
 ;   (terminated)
 ; ))

 (:goal (and
  (or
  ;  (confirmed left_wing )
    (confirmed right_wing)
    (status right_wing success)
  ;  (status left_wing success)
  )
  (terminated)

 ))
  (:goal-reward 100) (:metric maximize (reward))
)

