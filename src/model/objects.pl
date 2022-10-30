% This header defines an module with one predicate name has_ingredient that get's 2 
% arguments
:- module(objects,
	[
		is_pizza/1,
		has_ingredient/2
	]).

% If you want to load libraries and modules use:
%:- use_module(library('libraryname')).

% Knowrob allows to define specific methods for writing predicates specifically for
% projecting (writing into the knowledgebase) or requesting knowledge from the knowledgebase. 
% This is useful if the projecting logic is different to the requesting knowledge.
%
% For requesting knowledge use ?> instead of :-, for "projection" use +> instead of :-

% Prolog predicates are documented arcoding to: https://www.swi-prolog.org/pldoc/man?section=modes
%% is_pizza(?Entity) is det.
%
% True iff Entity is an instance of krexample:'Pizza'.
%
% @param Entity An entity IRI.
%
is_pizza(Entity) ?>
	% For asking if the entity is an pizza this call is enough:
	has_type(Entity, krexample:'Pizza').

is_pizza(Entity) +>
	% For creating a new object of type Pizza, we need to first generate
	% a new iri (the "name" of the object in the knowledgebase)
	new_iri(Entity, krexample:'Pizza'),
	has_type(Entity, krexample:'Pizza').


% The following is an example of getting data from the class of an individual:

%% has_ingredient(+Dish, ?Ingredient) is det.
%
% True iff Dish has an ingredient of type 
% Ingredient
%
% @param Dish An entity IRI.
% @param Ingredient An class of the Ingredient of the Dish
%
has_ingredient(Dish, Ingredient) ?>
	% Get the class of the entity
	has_type(Dish, DishType),
	% The class is subclass of a restriction (see the modelling in the owl file)
	subclass_of(DishType, Restriction),	
	is_restriction(Restriction, some(krexample:'hasIngredient', Ingredient)).


