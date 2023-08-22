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


Querying:
```
?- is_pizza(P).
P: http://www.ease-crc.org/ont/knowrob-example#Pizza_IOTQDJZX.
```

After creating a Pizza we can query for it's ingredients using `has_ingredient`:

```
?- is_pizza(P), has_ingredient(P,Ingredient).
Ingredient: http://www.ease-crc.org/ont/knowrob-example#Cheese,
P: http://www.ease-crc.org/ont/knowrob-example#Pizza_DXLUMEJP ;

Ingredient: http://www.ease-crc.org/ont/knowrob-example#Tomato,
P: http://www.ease-crc.org/ont/knowrob-example#Pizza_DXLUMEJP.
```
