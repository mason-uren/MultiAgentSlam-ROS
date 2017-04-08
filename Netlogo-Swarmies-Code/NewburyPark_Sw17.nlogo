;----------------------------------------------------------------------------------------------
 ;; COMPETITION SUBMISSION FILE
;----------------------------------------------------------------------------------------------
 ;; Elizabeth E. Esterly
 ;; elizabeth@cs.unm.edu
 ;; The University of New Mexico
 ;; Swarmathon 5: Competition
 ;; version 1.0
 ;; Last Revision 01/13/2017
;----------------------------------------------------------------------------------------------
  ;;
  ;;Newbury Park High School
  ;;Jason Isaacs and Richard Kick
  ;;Victoria Juan, Aaron Fang, Leon Xu, Akash Velu, Jack Smith, Nathan Juan, Jeffrey Sun, Andrew Reddy
;----------------------------------------------------------------------------------------------

;------------------------------------------------------------------------------------
;::::::::::::::::::::!!!!!!!!!   REQUIRED CODE  !!!!!!!!!::::::::::::::::::::
;------------------------------------------------------------------------------------
 ;;Use the bitmap extension.
 extensions[bitmap]

  patches-own [
     baseColor
     ;;You may add additional patches-own variables below this comment.

    ]

  turtles-own
  [
    ; state variables for the robot
    ; only 1 of these will be "true", the other is "false", at any given time
    searching? ; the robot is searching for tags? true/false
    returning? ; the robot is returning to the base? true/false
    returned? ; the last rock has been returned to base, so all robots need to return to base
    recruited?
    recruiting?
    targetX
    targetY
    pathX
    pathY
    pathDir ; heading right before leaving the path
    max-step-count
    step-count
    worth ; is the area worth looking over again
    reached? ; have you reached the target destination
    cluster? ; is there a cluster
    walkBack? ; are you trying to go back to your pattern path
    beginning? ; if just started, walk forward until the edge is reached
    first?
    beginningSteps
    abuse? ;;random search

  ]

;------------------------------------------------------------------------------------
;::::::::::::::::::::!!!!!!!!!   REQUIRED CODE  !!!!!!!!!::::::::::::::::::::
;------------------------------------------------------------------------------------
;------------------------------------------------------------------------------------
 ;;;;;;;;;;;;;;;;;;
 ;;    setup     ;; : MAIN PROCEDURE
 ;;;;;;;;;;;;;;;;;;
 ;------------------------------------------------------------------------------------
;;Write the setup procedure.
to setup
  file-config
   ;;You may add additional setup commands below this comment.

  ;;Create 6 turtles with basic shape
  create-turtles 6
  [
    set shape "robot"
    set size 8
    set searching? true ;;is the robot looking for rocks?
    set returning? false ;;is the robot going back to base?
    set returned? true ;;has the robot reached base?

    set recruiting? false ;;is the robot recruiting fellow robots?
    set recruited? false ;;has the robot been recruited by fellow robots?
    set worth 0 ;;should the robot continue searching in the specific area?
    set reached? false ;;has the robot reached the specific area desired?
    set walkBack? true ;;is the robot going home forever?
    set cluster? false ;;is the robot going to a cluster?
    set beginning? true ;;is the robot at the start of the search?
    set heading (who * 90 + 45) ;;what direction does the robot face?
    set step-count 0
    ifelse who < 2
    [ set max-step-count 200 ]
    [
      set max-step-count 200 - who * 10
    ]
    set beginningSteps max-step-count / (sqrt 2)
    set pathX 1000 ;;x position of path left
    set pathY 1000 ;;y position of path left
    set targetX 1000 ;;x position of specific area
    set targetY 1000 ;;y position of specific area
    set first? true ;;is the robot the first?
    set abuse? true ;;is the robot randomly searching now?
  ]

end

;------------------------------------------------------------------------------------
;::::::::::::::::::::!!!!!!!!!   REQUIRED CODE  !!!!!!!!!::::::::::::::::::::
;------------------------------------------------------------------------------------
 ;;;;;;;;;;;;;;;;;;;;;;;;;
 ;;    ROBOT CONTROL    ;; : MAIN PROCEDURE
 ;;;;;;;;;;;;;;;;;;;;;;;;;
 ;------------------------------------------------------------------------------------
 ;;Write the robot-control procedure.
to robot-control

  ; if all of the tags are collected, stop the model

    ask turtles
    [
      if searching?
      [ search ]

      if returning?
      [ return-to-base ]

      if recruiting?
      [ call-friends ]

      if recruited?
      [ go-to-friend ]
    ]

  ;;tick needs to come last in your robot-control procedure.
  ;------------------------------------------------------------------------------------
   tick
end

 ;;;;;;;;;;;;;;;;;;
;;CHECK FOR TAGS;;
;;;;;;;;;;;;;;;;;;

to check-for-tags
;;look nearby and see if there are tags to pick up. If there are lots of tags in the area, call nearby robots to
;;come and fetch the tags too
    if count neighbors in-radius 2 with [ pcolor = yellow ] > 0
    [
      if searching?
      [
        ask one-of neighbors in-radius 2 with [ pcolor = yellow ]
        [ set pcolor baseColor ]
        set shape "robot with rock"
        set searching? false

        ;;check that the robot is not already occupied with a different task - walking back to home or have gotten recruited

        if not walkBack? and not recruited?
        [
          if worth <= 0
          [
            set pathX pxcor
            set pathY pycor
            set pathDir heading
          ]
        ]

        ;;call other robots to get recruited

        let isRecruiting? false
        let isReturning? false

        ifelse count patches in-radius 2 with [ pcolor = yellow ] > 6
        [ set isRecruiting? true ]
        [ set isReturning? true ]

        set recruiting? isRecruiting?
        set returning? isReturning?

        if recruiting?
        [
          set worth 80
          set reached? false
          set targetX pxcor
          set targetY pycor
        ]
      ]
    ]

end

;;;;;;;;;;
;;SEARCH;;
;;;;;;;;;;

to search

  let xvals pxcor = min-pxcor + 1 or pxcor = max-pxcor - 1
  let yvals pycor = min-pycor + 1 or pycor = max-pycor - 1
  let corner xvals or yvals
  ifelse (corner and abuse?)
  [
    ifelse (xvals and yvals)
    [
      ;;robot is hitting the wall in initial setup, fix
      set first? false
      left 45
      set abuse? false
    ]
    [
      if step-count < 0
      [ facexy 0 0 ] ;;
    ]
    forward 1
  ]
  [
    if not beginning? [ check-for-tags ] ;;robot can check for tags if not engaged in initial setup
    forward 1

    ifelse reached?
    [
      if worth > 0
      [
        ifelse distancexy targetX targetY > 20 ;;robot should be engaging in heightened randomized search near cluster area, but is going out of range
        [
          set abuse? true
          faceXY targetX targetY
        ]
        [
          ;;robot should be engaging in heightened randomized search near cluster area, is in range
          set abuse? true
          left random 20
          right random 20
        ]
        set worth (worth - 1)
        if worth = 0
        [
          ;;robot should no longer be engaging in heightened randomized search near cluster area; return to path
          set cluster? false
          set abuse? false
          set reached? false
          set walkBack? true
        ]
      ]
    ]
    [
      ifelse worth > 0
      [
        ifelse targetX = pxcor and targetY = pycor
        [ set reached? true ] ;;robot has arrived at cluster, will now begin to do heightened randomized search
        [ facexy targetX targetY ] ;;robot has not arrived at cluster yet, is heading there
      ]
      [
        turning
      ]
    ]
  ]

end

;;;;;;;;;;;;;;;;;;
;;RETURN TO BASE;;
;;;;;;;;;;;;;;;;;;

to return-to-base

  ;;procedure to go back to the base in the center

  ;;if the robot has already returned, reverse necessary variables
  ifelse count neighbors in-radius 2 with [ pcolor = green ] > 0 and
         distancexy 0 0 < 1.5
  [
    ifelse returned?
    [
      set returned? false
    ]
    [
      set shape "robot"
      set returning? false
      if not cluster? [ set walkBack? true ]
      if worth > 0
      [ set reached? false ]
      set searching? true
    ]
  ]
  [
    ;;otherwise the robot has not reached the center base. continue walking towards the origin
    face-origin
    forward 1
  ]

end

;;;;;;;;;;;;;;;;
;;CALL FRIENDS;;
;;;;;;;;;;;;;;;;

to call-friends

  ;;if other robots are being recruited, this procedure is called. set the proper variables
  let rockX pxcor
  let rockY pycor

  ask turtles in-radius 2
  [
    if searching?
    [
      set searching? false
      set recruited? true
      set reached? false
      set cluster? true

      set targetX rockX
      set targetY rockY

      ;;if the seraching is done, then go back to the path search
      if worth <= 0 and (not walkBack? and not recruited?)
      [
        set pathX pxcor
        set pathY pycor
        set pathdir heading
      ]
    ]
  ]
  set recruiting? false
  set returning? true
end

;;;;;;;;;;;;;;;;
;;GO TO FRIEND;;
;;;;;;;;;;;;;;;;

to go-to-friend
  ;;if recruited, head towards the special spot/face the special spot
  if not first?
  [
    if not (worth > 0) and not walkBack?
    [
      set pathX pxcor
      set pathY pycor
      set pathDir heading
    ]
    set label "recruited"
    set cluster? true
    ifelse targetX = pxcor and targetY = pycor
    [
      set searching? true
      set recruited? false
      set reached? true
      set label ""
    ]
    [
      set worth 80
      facexy targetX targetY
      forward 1
    ]
  ]
end

;;;;;;;;;;;
;;TURNING;;
;;;;;;;;;;;

to turning
  ;;programmed path algorithm: robots are programmed to go in a square search around the arena in gradually smaller squares
  ifelse walkBack?
  [
    ifelse beginning?
    [
      set beginningSteps beginningSteps - 1
      if beginningSteps <= 1
      [
        set beginning? false
        set walkBack? false
      ]
    ]
    [
      ifelse pathX = pxcor and pathY = pycor
      [
        set walkBack? false
        set heading pathDir
      ]
      [
        if pathX != 1000 and pathY != 1000
        [ facexy pathX pathY ]
      ]
    ]
  ]
  [
    ifelse step-count > 0
    [
      set step-count step-count - 1
    ]
    [
      ifelse step-count = 0
      [
        if first?
        [
          set first? false
          left 45
        ]
        left 90
        set max-step-count (max-step-count - 1)
        set step-count max-step-count
      ]
      [
        ;;if the pathway search has been exhausted, just do a random search
        set abuse? true
        left random 20
        right random 20
      ]
    ]
  ]
end

to face-origin
  facexy 0 0
end

;;KEEP THE FOLLOWING CODE AT THE BOTTOM OF YOUR FILE.
;------------------------------------------------------------------------------------
;::::::::::::::::::::!!!!!!!!!   REQUIRED CODE  !!!!!!!!!::::::::::::::::::::
;------------------------------------------------------------------------------------
 ;;;;;;;;;;;;;;;;;;;;;;;
 ;;   file-config     ;;
 ;;;;;;;;;;;;;;;;;;;;;;;
 ;------------------------------------------------------------------------------------
;;
 to file-config
  ca ;clear all
  cp ;clear patches
  bitmap:copy-to-pcolors bitmap:import "parkingLot.jpg" true
  reset-ticks ;keep track of simulation runtime
  make-rocks
  make-base
 end

;------------------------------------------------------------------------------------
to make-rocks
   ask patches [ set baseColor pcolor]

   if distribution = "cross" or distribution = "random + cross" or distribution = "large clusters + cross"
   or distribution = "clusters + cross" or distribution = "random + clusters + cross"
   or distribution = "random + clusters + large clusters + cross"[make-cross]

   if distribution = "random" or distribution = "random + cross" or distribution = "random + clusters"
   or distribution = "random + large clusters" or distribution = "random + clusters + cross"
   or distribution = "random + clusters + large clusters + cross" [make-random]

   if distribution = "clusters" or distribution = "clusters + cross" or distribution = "random + clusters"
   or distribution = "clusters + large clusters" or distribution = "random + clusters + cross"
   or distribution = "random + clusters + large clusters + cross" [make-clusters]

   if distribution = "large clusters" or distribution = "large clusters + cross"
   or distribution = "random + large clusters"  or distribution = "clusters + large clusters"
   or distribution = "random + clusters + large clusters + cross" [make-large-clusters]

end

;------------------------------------------------------------------------------------
;;Place rocks in a cross formation.
to make-cross
  ask patches [
    ;;Set up the cross by taking the max coordinate value, doubling it, then only setting a rock if the
    ;;x or y coord is evenly divisible by that value.
    ;;NOTE: This technique assumes a square layout.
    let doublemax max-pxcor * 2
    if pxcor mod doublemax = 0 or pycor mod doublemax = 0 [ set pcolor yellow ]
  ]
end

;------------------------------------------------------------------------------------
;;Place rocks randomly.
to make-random
   let targetPatches singleRocks
     while [targetPatches > 0][
       ask one-of patches[
         if pcolor != yellow[
           set pcolor yellow
           set targetPatches targetPatches - 1
         ]
       ]
     ]
end

;------------------------------------------------------------------------------------
;;Place rocks in clusters.
to make-clusters
   let targetClusters clusterRocks
     while [targetClusters > 0][
       ask one-of patches[
         if pcolor != yellow and [pcolor] of neighbors4 != yellow[
           set pcolor yellow
           ask neighbors4[ set pcolor yellow ]
           set targetClusters targetClusters - 1
         ]
       ]
     ]
end

;------------------------------------------------------------------------------------
;;Place rocks in large clusters.
to make-large-clusters
   let targetLargeClusters largeClusterRocks
   while [targetLargeClusters > 0][
     ask one-of patches[
       if pcolor != yellow and [pcolor] of patches in-radius 2 != yellow[
         set pcolor yellow
         ask patches in-radius 2 [set pcolor yellow]
         set targetLargeClusters targetLargeClusters - 1
       ]
     ]
     ]
end

;------------------------------------------------------------------------------------
;Make a base at the origin.
to make-base
  ask patches[
    if distancexy 0 0 = 0 [set pcolor green]
  ]
end
@#$#@#$#@
GRAPHICS-WINDOW
235
13
750
549
50
50
5.0
1
10
1
1
1
0
0
0
1
-50
50
-50
50
0
0
1
ticks
5.0

BUTTON
14
10
81
44
setup
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
89
11
207
45
robot-control
robot-control
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

MONITOR
15
60
129
105
rocks remaining
count patches with [pcolor = yellow]
17
1
11

CHOOSER
15
323
232
368
distribution
distribution
"cross" "random" "clusters" "large clusters" "random + cross" "clusters + cross" "clusters + large clusters" "large clusters + cross" "random + clusters" "random + large clusters" "random + clusters + cross" "random + clusters + large clusters + cross"
11

SLIDER
17
199
189
232
singleRocks
singleRocks
0
100
45
5
1
NIL
HORIZONTAL

SLIDER
17
237
189
270
clusterRocks
clusterRocks
0
50
25
5
1
NIL
HORIZONTAL

SLIDER
17
278
189
311
largeClusterRocks
largeClusterRocks
0
20
3
1
1
NIL
HORIZONTAL

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

robot
true
1
Polygon -16777216 true false 75 60 105 15 135 15 90 75 75 60
Polygon -16777216 true false 225 60 195 15 165 15 210 75 225 60
Rectangle -16777216 true false 30 105 75 165
Rectangle -16777216 true false 30 210 75 270
Rectangle -16777216 true false 75 120 225 255
Rectangle -16777216 true false 225 210 270 270
Rectangle -16777216 true false 225 105 270 165
Rectangle -16777216 true false 90 120 210 195
Rectangle -2674135 true true 120 120 180 240
Rectangle -16777216 true false 195 225 210 240
Rectangle -16777216 true false 120 60 180 90
Rectangle -16777216 true false 135 90 165 120
Polygon -2674135 true true 180 75 210 90 210 105 180 90 180 75
Polygon -2674135 true true 120 75 90 90 90 105 120 90 120 75
Rectangle -7500403 true false 75 105 225 120
Line -2674135 true 90 120 90 240
Line -2674135 true 210 120 210 240
Line -2674135 true 135 90 165 90
Line -16777216 false 75 165 225 165
Line -16777216 false 75 180 225 180
Rectangle -7500403 true false 75 240 225 255

robot with rock
true
1
Rectangle -1184463 true false 120 0 180 45
Polygon -16777216 true false 75 60 105 15 135 15 90 75 75 60
Polygon -16777216 true false 225 60 195 15 165 15 210 75 225 60
Rectangle -16777216 true false 30 105 75 165
Rectangle -16777216 true false 30 210 75 270
Rectangle -16777216 true false 75 120 225 255
Rectangle -16777216 true false 225 210 270 270
Rectangle -16777216 true false 225 105 270 165
Rectangle -16777216 true false 90 120 210 195
Rectangle -2674135 true true 120 120 180 240
Rectangle -16777216 true false 120 60 180 90
Rectangle -16777216 true false 135 90 165 120
Polygon -2674135 true true 180 75 210 90 210 105 180 90 180 75
Polygon -2674135 true true 120 75 90 90 90 105 120 90 120 75
Rectangle -7500403 true false 75 105 225 120
Line -2674135 true 75 240 225 240
Line -2674135 true 90 120 90 240
Line -2674135 true 210 120 210 240
Line -2674135 true 135 90 165 90
Line -16777216 false 75 165 225 165
Line -16777216 false 75 180 225 180
Rectangle -7500403 true false 75 240 225 255

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.2.1
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
