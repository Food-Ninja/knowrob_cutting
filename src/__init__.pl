% You can register other knowrob packages here. 
:- register_ros_package(knowrob).
:- register_ros_package(knowrob_KG_cutting).

:- use_module(library(semweb/rdf_db)), 
   use_module(library(sparqlprog)), 
   use_module(library(semweb/rdfs)), 
   use_module(library(sparqlprog/ontologies/owl)).

% Load the knowledge graph and define a name to be used later (fc)
:- sparql_endpoint(fc, 'https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql').

% Register prefixes that can be used later, e.g.
% we can write SOMA:Cutting instead of the full URI
/** PREFIX MAPPING */
:-rdf_register_prefix(cut, 'http://www.ease-crc.org/ont/food_cutting#')
:-rdf_register_prefix(SOMA, 'http://www.ease-crc.org/ont/SOMA.owl#')
:-rdf_register_prefix(cut2, 'http://www.ease-crc.org/ont/situation_awareness#')
:-rdf_register_prefix(DUL, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#')
:-rdf_register_prefix(obo, 'http://purl.obolibrary.org/obo/')
:-rdf_register_prefix(owl, 'http://www.w3.org/2002/07/owl#')

% Load our owl file and define a namespace that can be used later, e.g.
% in model/objects.pl we write knowrobexample:Pizza instead of the full
% URI ('http://www.ease-crc.org/ont/knowrob-example#Pizza')
%:- load_owl('package://knowrob_example/owl/knowrob-example.owl',
%	[namespace(krexample,'http://www.ease-crc.org/ont/knowrob-example#')]).

% Load your subdirectories here
:- use_directory('util').              % Accessing the position for cutting
:- use_directory('cuttingparams').     % Retrieving tool to be used, prior cutting tasks to be executed and number of repetitions
