# knowrob_KG_cutting

This is a documented example of a KnowRob Package accessing a knowledge graph and offering modules for querying the knowledge graph for execution of food cutting actions. 

In contrast to other knowrob packages, it doesn't include an owl directory since we access a knowledge graph.
The knowledge graph is available at <a href='https://krr.triply.cc/mkumpel/ProductKG/sparql/ProductKG'>triply</a>

In 'src' you can find the source code of this package. It contains a util folder for retrieving the position needed for a cutting action as well as a params folder for retrieving additional parameters needed for performing different cutting actions.

## Example run:

To run the package use the launch file

```
roslaunch knowrob_KG_cutting knowrob_KG_cutting.launch
```

You can then use the following command to send queries:

```
rosrun rosprolog rosprolog_commandline.py
```

In 'src/util/pose.pl' we defined `position_to_be_used`, as a simple example of retrieving the pose needed for cutting (it only differentiates between slicing_position and halving_position, the robot can further infer the actual position according to this info). 

```
?- position_to_be_used(SOMA:'Cutting',Pose).
Pose: slicing_position
```

In 'src/model/objects.pl' we define predicates to query object and task knowledge from the knowledge base. 

For example, one can query the tool to be used for cutting a specific food object.
Querying:
```
?- tool_to_be_used(obo:'FOODON_00003415',Tool).
Tool: Knife.
```

Or you can query if a prior action needs to be executed (For quartering, we first perform halving).
```
?- retrieve_prior_action(SOMA:'Dicing', PriorAction).
PriorAction: Julienning.
```
You can also query for additional actions that need to be performed (an orange needs to be peeled before cutting)
```
retrieve_additional_action(obo:'FOODON_03309927', Action).
Action: StemRemoving
```
