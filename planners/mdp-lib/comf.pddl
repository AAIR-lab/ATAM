(define (domain keva)
  (:requirements :strips :typing :fluents :conditional-effects)
  
  (:types robot plank location)
  (:predicates
      (onTable ?p - plank)
      (onSinglePlank ?p - plank ?p - plank)
      (onDoublePlank ?p - plank ?p - plank ?p - plank)
      (clearPlank ?p - plank)
      (handEmpty)
      (inGripper ?p - plank)
      (placed ?p - plank)
      (human_placed ?p - plank ?loc - location)
      (free ?loc - location)
  )

  (:functions 
    (amount ?p - plank)
  )

  (:constants 
           plank1 plank2 plank3 plank4 - plank
           location1 location2 - location
  )

  (:action pickUp_plank_from_region
  		 :parameters (?rob - robot ?p - plank ?loc - location)
  		 :precondition 
          (and
            (handempty) 
            (clearPlank ?p)
            (human_placed ?p ?loc)
            (not (free ?loc))
          )
  		 :effect (and (not (handempty)) 
                      (inGripper ?p)
                      (not (clearPlank ?p))
                      (not (onTable ?p))
                      (not (human_placed ?p ?loc))
                      (free ?loc)
                      (forall (?pl - plank)
                            (when (and (onSinglePlank ?p ?pl)
                                       (not (= ?p ?pl))
                                  )
                                  (and (not(onSinglePlank ?p ?pl))
                                       (clearPlank ?pl)
                                  )
                            )
                      )
                )
  )



  (:action putDown_plank_onTable
  		 :parameters (?rob - robot ?p - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p)
                            (not (placed ?p)))
  		 :effect (and (handempty)
                      (onTable ?p)
                      (clearPlank ?p)
                      (not (inGripper ?p))
                      (placed ?p))
  )



  (:action putDown_plank_onSinglePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p1)
                            (not (= ?p1 ?p2))
                            (not (placed ?p1))
                            (placed ?p2))
  		 :effect (and (handempty)
                      (onSinglePlank ?p1 ?p2)
                      (not (inGripper ?p1))
                      (not (clearPlank ?p2))
                      (clearPlank ?p1)
                      (placed ?p1))
  )



  (:action putDown_plank_onDoublePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank ?p3 - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p1)
                            (not (= ?p1 ?p2))
                            (not (= ?p1 ?p3))
                            (not (= ?p2 ?p3))
                            (not (placed ?p1))
                            (placed ?p2)
                            (placed ?p3)
                      )
  		 :effect (and (handempty)
                      (not (inGripper ?p1))
                      (onDoublePlank ?p1 ?p2 ?p3)
                      (not (clearPlank ?p2))
                      (not (clearPlank ?p3))
                      (clearPlank ?p1)
                      (placed ?p1))
  )


  (:action human_place
    :parameters (?p - plank)
    :precondition (and
    	(handempty)
        (free location1)
        (free location2)

    )
    :effect (and
        (probabilistic
            0.4 (and (human_placed ?p location1)(not (free location1))(clearPlank ?p))
            0.6 (and (human_placed ?p location2)(not (free location2))(clearPlank ?p))
        )
    )
  )

  (:action done  
    :precondition ( and
        (onTable plank1)
        (onTable plank2)
        (onDoublePlank plank3 plank1 plank2)
        (onDoublePlank plank4 plank1 plank2)
    )

    :effect (and
       (terminated)
    )
  )
)
(define (problem p01)

	(:domain keva)
	(:objects
		plank1 plank2 plank3 plank4 - plank
		location1 location2 - location
 		yumi - robot
	)

	(:init
		(handempty)
		(free location1)
		(free location2)
	)

	(:goal
		(terminated)
	)
)