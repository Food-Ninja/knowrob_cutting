% This header defines an module with predicates that get /number of params as input
:- module(objects,
	[
		tool_to_be_used/2,
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
tool_to_be_used(Food,Tool):- fc ??
  rdfs_subclass_of(Node, Food), rdf(Node, owl:'onProperty', SOMA:'hasDisposition'), rdf(Node, owl:'someValuesFrom', NodeTwo), 
  rdf(NodeTwo, owl:'intersectionOf', Intersect), rdf(Intersect, rdf:'first', cut2:'Cuttability'), rdf(Intersect, rdf:'rest', NodeThree),
  rdf(NodeThree, rdf:'rest', NodeFour), rdf(NodeFour, rdf:'first', Toolnode), rdf(Toolnode, owl:'onProperty', SOMA:'affordsTrigger'),
  rdf(Toolnode, owl:'allValuesFrom', SomeTool), rdf(SomeTool, owl:'onProperty', DUL:'classifies'), rdf(SomeTool, owl:'allValuesFrom', Alltools),
  rdfs_subclass_of(Tool, Alltools).




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


