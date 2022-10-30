% This header defines an module with one predicate name get_better_pose that get's 2 
% arguments
:- module(pose,
    [ 
      get_better_pose/2
    ]).

%% get_better_pose(+Pose, ?NewPose) is det.
%
% Example of calculating a new pose from a given pose.
%
% @param Pose The pose to be manipulated
% @param NewPose The calculated pose
%
get_better_pose([[X,Y,Z],Rotation],[[XNew,Y,Z],Rotation]) :- 
  XNew is X + 1.0.