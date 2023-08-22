% You can register other knowrob packages here. 
:- register_ros_package(knowrob).
:- register_ros_package(knowrob_KG_cutting).

% Load our owl file and define a namespace that can be used later, e.g.
% in model/objects.pl we write knowrobexample:Pizza instead of the full
% URI ('http://www.ease-crc.org/ont/knowrob-example#Pizza')
%:- load_owl('package://knowrob_example/owl/knowrob-example.owl',
%	[namespace(krexample,'http://www.ease-crc.org/ont/knowrob-example#')]).

% Load your subdirectories here
:- use_directory('util').
:- use_directory('model').
