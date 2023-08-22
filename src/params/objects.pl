% This header defines an module with predicates that get /number of params as input
:- module(objects,
	[
		tool_to_be_used/1,
	]).
 
% If you want to load libraries and modules use:
%:- use_module(library('libraryname')).

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



% Knowrob allows to define specific methods for writing predicates specifically for
% projecting (writing into the knowledgebase) or requesting knowledge from the knowledgebase. 
% This is useful if the projecting logic is different to the requesting knowledge.
%
% For requesting knowledge use ?> instead of :-, for "projection" use +> instead of :-

% Prolog predicates are documented arcoding to: https://www.swi-prolog.org/pldoc/man?section=modes
%
%
% @param Food A food with FoodOn identifier, e.g. obo:FOODON_03301710 (apple)
%
/**What tool can be used for cutting the given food? Input Food: z.B. obo:FOODON_03301710 (Apfel)*/
tool_to_be_used(Food,Tool)?> fc ??
  rdfs_subclass_of(?node, Food), rdf(?node, owl:'onProperty', SOMA:'hasDisposition'), rdf(?node, owl:'someValuesFrom', ?a), 
  rdf(?a, owl:'intersectionOf', ?b), rdf(?b, rdf:'first', cut2:'Cuttability'), rdf(?b, rdf:'rest', ?c),
  rdf(?c, rdf:'rest', ?node2), rdf(?node2, rdf:'first', ?toolnode), rdf(?toolnode, owl:'onProperty', SOMA:'affordsTrigger'),
  rdf(?toolnode, owl:'allValuesFrom', ?tool), rdf(?tool, owl:'onProperty', DUL:'classifies'), rdf(?tool, owl:'allValuesFrom', ?alltools),
  rdfs_subclass_of(Tool, ?alltools).




% The following is an example of getting data from the class of an individual:

%% has_ingredient(+Dish, ?Ingredient) is det.
%
% True iff Dish has an ingredient of type 
% Ingredient
%
% @param Dish An entity IRI.
% @param Ingredient An class of the Ingredient of the Dish
%
%has_ingredient(Dish, Ingredient) ?>
	% Get the class of the entity
%	has_type(Dish, DishType),
	% The class is subclass of a restriction (see the modelling in the owl file)
%	subclass_of(DishType, Restriction),	
%	is_restriction(Restriction, some(krexample:'hasIngredient', Ingredient)).


