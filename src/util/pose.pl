% This header defines a module with one predicate name get_better_pose that gets 1 
% argument
:- module(pose,
    [ 
      position_to_be_used/1
    ]).

%% get_better_pose(+Pose, ?NewPose) is det.
%
% /**What position is needed for cutting? Input Action: z.B. SOMA:Dicing. Output: either slicing_position or halving_position*/
%
% @param Action The action to be performed (cutting, halving, quartering, slicing, julienning, dicing)
% @param Pose The pose to be used
%
position_to_be_used(Action,Pose):- fc ??
  rdfs_subclass_of(?node, Action), rdf(?node, owl:'onProperty', cut:'requiresPosition'), rdf(?node, owl:'someValuesFrom', Pose).
