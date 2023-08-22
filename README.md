# knowrob_KG_cutting

This is a documented example of a KnowRob Package accessing a knowledge graph and offering modules for querying the knowledge graph for execution of food cutting actions. 

In 'src' you will find the source code of this package. 

## Example run:

To run the package use the launch file

```
roslaunch knowrob_KG_cutting knowrob_KG_cutting.launch
```

You can then use the following command to send queries:

```
rosrun rosprolog rosprolog_commandline.py
```

In 'src/util/pose.pl' we defined `get_better_pose`, as a simple example of
manipulation of a pose. 

```
?- get_better_pose([[1.0,1.0,1.0],[0.0, 0.0, 0.0, 1.0]],[[XNew,Y,Z],Rotation]).
Rotation: [0.0, 0.0, 0.0, 1.0],
XNew: 2.0,
Y: 1.0,
Z: 1.0.
```

In 'src/model/objects.pl' we define predicates to query and manipulate knowledge in the knowledge base. 

To create new knowledge you can use kb_project (different to assert `kb_project` will project the knowledge into the mongo database). The simplest way would be to assert a new triple into the knowledge base (e.g. `kb_project(triple(a,b,c))`). In objects.pl we defined `is_pizza` that allows to create and query new pizza entities:

Creating:
```
?- kb_project(is_pizza(P)).
P: http://www.ease-crc.org/ont/knowrob-example#Pizza_IOTQDJZX.
```

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
