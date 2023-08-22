% This header defines a module with one predicate name get_better_pose that gets 1 
% argument
:- module(pose,
    [ 
      position_to_be_used/1
    ]).

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

%% get_better_pose(+Pose, ?NewPose) is det.
%
% /**What position is needed for cutting? Input Action: z.B. SOMA:Dicing. Output: either slicing_position or halving_position*/
%
% @param Action The action to be performed (cutting, halving, quartering, slicing, julienning, dicing)
% @param Pose The pose to be used
%
position_to_be_used(Action,Pose)?> fc ??
  rdfs_subclass_of(?node, Action), rdf(?node, owl:'onProperty', cut:'requiresPosition'), rdf(?node, owl:'someValuesFrom', Pose).
