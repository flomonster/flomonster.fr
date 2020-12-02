Introduction
------------

The purpose of this article is to describe how to implement an optimized KD tree
for raytracing (or other rendering engine) in Rust. The choice of language is
arbitrary and should not disturb a *non-rustacean* (who doesn't code in rust) reader.

The implementation I will use is described in the scientific paper `On building fast kd-Trees
for Ray Tracing, and on doing that in O(N log N)
<http://www.irisa.fr/prive/kadi/Sujets_CTR/kadi/Kadi_sujet2_article_Kdtree.pdf>`_.
Since this paper describes well the theoretical part, I will focus on the
implementation. However, if you just want to implement a KD tree without diving
into the paper formulas, you can keep reading without woring.

Each version of the code is available:

* Naive implementation: `<https://github.com/flomonster/kdtree-ray/tree/naive>`_.
* Sah in :math:`O(N^2)`: `<https://github.com/flomonster/kdtree-ray/tree/sah-quadratic>`_.
* Sah in :math:`O(N \log^2{N})`: `<https://github.com/flomonster/kdtree-ray/tree/sah-log2>`_.

The models used for benchmark are from the `Stanford Scanning Repository
<http://graphics.stanford.edu/data/3Dscanrep/>`_.

General principles
------------------

Context
=======

Many rendering engines are based on the physics of light rays. This method allows
to obtain realistic images, but it is quite slow. Indeed, it requires throwing a
lot of rays and checking if they intersect an object in the scene.

To do this, the easiest way is to check for each primitive (for example triangles)
of the scene if it intersects the ray. Even if you have an efficient way to check
this intersection, it cannot be viable for a scene with billions of primitives.

The most common solution to this problem is the use of KD tree data structures.

KD what?
========

Theory
######

KD tree is a data structure that organize a k-dimensional space. In our case the
``k`` is ``3`` since we want to organize the space of our 3D scenes.

KD trees are **full binary trees**. This means that each internal node contains
exactly two children.

They are formed as follows:

- **Internal nodes** divide the space in half in a selected dimension. They
  create a sub-space for each child.
- **Leaves** contain a set of objects that are in their sub-space.

.. figure:: /img/articles/kdtree/3dtree.png
   :width: 50%
   :alt: A 3-dimensional space organized by k-d tree.

   A 3-dimensional space organized by k-d tree. The first split (the red vertical
   plane) cuts the root cell (white) into two subcells, each of which is then split
   (by the green horizontal planes) into two subcells. Finally, four cells are
   split (by the four blue vertical planes) into two subcells. Since there is no
   more splitting, the final eight are called leaf cells.

How to build it?
################

Remember that the goal is to know which primitives intersect a ray without having
to check each of them. The idea is to provide a reduced list of primitive
candidates that could intersect a ray. So the quality of our KD tree depends on
three things:

- The number of candidates returned by our KD tree.
- The time taken by the KD tree to generate the list.
- The time taken to create the KD tree. This point can be considered less
  important since the tree is built only once.

During our construction, we will have to check if the primitives intersect a
sub-space or not to be able to arrange them in the right node of the tree.
To do so sub-space and primitives will be described by a 3D **AABB**
(Axis-aligned bounding boxes).

.. figure:: /img/articles/kdtree/aabb.gif
   :alt: An animated representation of AABB.

   An AABB that adapts its size to fit an entity. (`source <https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection>`_)

An AABB is convenient and optimized to check if two entities overlap. It is also
simple to check if a ray intersects an AABB.

So, to build a KD tree, we must recursively divide a space and classify which
primitives overlap the new subspaces. To optimize the above criteria, we must
divide the space optimally and stop recursion optimally.

Naive implementation
--------------------

This version will serve as a proof of concept. And yet, it will significantly
reduce the intersection search algorithm runtime.

The full code is available `here <https://github.com/flomonster/kdtree-ray/tree/naive>`_.

Bounding Box
============

First of all, we have to define our AABB since that's what we're going to
manipulate.

.. code:: rust

   use cgmath::*;

   /// Axis-aligned bounding box is defined by two positions.
   #[derive(Clone, Debug)]
   pub struct AABB(pub Vector3<f32>, pub Vector3<f32>);

Some function will be needed, as describe before:

- ``intersect_ray`` will check if a ray (described by an origin and a direction)
  intersect our AABB. `More info about the math <https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection>`_
- ``intersect_box`` will check if our AABB overlap with another AABB.

.. code:: rust

   impl AABB {
       pub fn intersect_ray(
           &self,
           ray_origin: &Vector3<f32>,
           ray_direction: &Vector3<f32>,
       ) -> bool {
           let mut tmin = (self.0.x - ray_origin.x) / ray_direction.x;
           let mut tmax = (self.1.x - ray_origin.x) / ray_direction.x;

           if tmin > tmax {
               std::mem::swap(&mut tmin, &mut tmax);
           }

           let mut tymin = (self.0.y - ray_origin.y) / ray_direction.y;
           let mut tymax = (self.1.y - ray_origin.y) / ray_direction.y;

           if tymin > tymax {
               std::mem::swap(&mut tymin, &mut tymax);
           }

           if (tmin > tymax) || (tymin > tmax) {
               return false;
           }

           tmin = tmin.max(tymin);
           tmax = tmax.min(tymax);

           let mut tzmin = (self.0.z - ray_origin.z) / ray_direction.z;
           let mut tzmax = (self.1.z - ray_origin.z) / ray_direction.z;

           if tzmin > tzmax {
               std::mem::swap(&mut tzmin, &mut tzmax);
           }

           if (tmin > tzmax) || (tzmin > tmax) {
               return false;
           }

           true
       }

       pub fn intersect_box(&self, other: &AABB) -> bool {
           (self.0.x < other.1.x && self.1.x > other.0.x)
               && (self.0.y < other.1.y && self.1.y > other.0.y)
               && (self.0.z < other.1.z && self.1.z > other.0.z)
       }
   }

Finally, we need a **trait** that our primitives will have to implement. So we are
sure to have an AABB for our primitives.

.. code:: rust

   pub trait BoundingBox {
       fn bounding_box(&self) -> AABB;
   }

KD Tree Structs
===============

Let's create our ``KDtree`` structure. It will contain the root node and an initial
space. The initial space is an AABB that contains all the primitives. It will
have to be computed during its construction.

.. code:: rust

   /// P is our primitive and has to implement the trait BoundingBox
   pub struct KDtree<P: BoundingBox> {
       root: KDtreeNode<P>,
       space: AABB,
   }

Now we can now define our ``KDtreeNode``. In rust ``enum`` are perfect for this
kind of object. It allows us to define two state:

- ``Leaf``: Represents a leaf of our tree.
- ``Node``: Represents an internal node of our tree.

.. code:: rust

   use std::sync::Arc;

   #[derive(Clone, Debug)]
   pub enum KDtreeNode<P: BoundingBox> {
       Leaf {
           space: AABB,
           values: Vec<Arc<P>>,
       },
       Node {
           left_space: AABB,
           left_node: Box<KDtreeNode<P>>,
           right_space: AABB,
           right_node: Box<KDtreeNode<P>>,
       },
   }

We are using ``Arc`` cause our primitive could be clone in several branches of our
tree. To avoid copying the full object ``Arc`` allows us to do reference counting.

Plan
====

Let's create a structure that represents a split in a space. Since our space is
in 3D a plan is perfect to represents this seperation.

.. code:: rust

   #[derive(Clone, Debug)]
   pub enum Plan {
       X(f32), // Split on the X-axis
       Y(f32), // Split on the Y-axis
       Z(f32), // Split on the Z-axis
   }


Item
====

Before starting the kdtree implementation we will define an Item structure that
will simplify our code.

An ``Item`` is simply the aggregation of a primitive and its AABB.

.. code:: rust

   #[derive(Debug)]
   pub struct Item<P: BoundingBox> {
       pub value: Arc<P>,
       pub bb: AABB,
   }

   impl<P: BoundingBox> Item<P> {
       /// Method to create a new Item from a primitive.
       pub fn new(value: P) -> Self {
           let bb = value.bounding_box();
           Item {
               value: Arc::new(value),
               bb,
           }
       }
   }

   /// Implementation of the Clone will be needed when our item will have to
   /// follow different branches of the tree.
   impl<P: BoundingBox> Clone for Item<P> {
       fn clone(&self) -> Self {
           Item {
               value: self.value.clone(),
               bb: self.bb.clone(),
           }
       }
   }

We can also define ``Items`` which is a list of ``Arc<Item>``.

.. code:: rust

   pub type Items<P> = Vec<Arc<Item<P>>>;

Build KD Tree
=============

KDtree
######

Let's first implement the function that build a ``KDtree``. To do so we need a list
of primitives. The function will compute the initial space of the KDtree and
create the root node.

.. code:: rust

   impl<P: BoundingBox> KDtree<P> {
     /// This function is used to create a new KD-tree. You need to provide a
     /// `Vec` of values that implement `BoundingBox` trait.
     pub fn new(mut values: Vec<P>) -> Self {
         let mut space = AABB(Vector3::<f32>::max_value(), Vector3::<f32>::min_value());
         let mut items = Items::new();
         while let Some(v) = values.pop() {
             // Create items from values
             let item = Arc::new(Item::new(v));

             // Update space with the bounding box of the item
             space.0.x = space.0.x.min(item.bb.0.x);
             space.0.y = space.0.y.min(item.bb.0.y);
             space.0.z = space.0.z.min(item.bb.0.z);
             space.1.x = space.1.x.max(item.bb.1.x);
             space.1.y = space.1.y.max(item.bb.1.y);
             space.1.z = space.1.z.max(item.bb.1.z);

             items.push(item);
         }
         let root = KDtreeNode::new(&space, items, 10);
         KDtree { space, root }
     }
   }

Note that the ``max_depth`` will allow us to create a stopping criterion easily.
The value was chosen arbitrarily.

KDtreeNode
##########

Let's implement the function to create a ``KDtreeNode``.

.. code:: rust

   impl<P: BoundingBox> KDtreeNode<P> {
     pub fn new(space: &AABB, mut items: Items<P>, max_depth: usize) -> Self {
         // Heuristic to terminate the recursion
         if items.len() <= 15 || max_depth == 0 {
             // Create the vector
             let mut values = vec![];
             while let Some(i) = items.pop() {
                 values.push(i.value.clone());
             }
             return Self::Leaf {
                 space: space.clone(),
                 values,
             };
         }

         // Find a plane to partition the space
         let plan = Self::partition(&space, max_depth);

         // Compute the new spaces divided by `plan`
         let (left_space, right_space) = Self::split_space(&space, &plan);

         // Compute which items are part of the left and right space
         let (left_items, right_items) = Self::classify(&items, &left_space, &right_space);

         Self::Node {
             left_node: Box::new(Self::new(&left_space, left_items, max_depth - 1)),
             right_node: Box::new(Self::new(&right_space, right_items, max_depth - 1)),
             left_space,
             right_space,
         }
     }
   }

Note that an arbitrary heuristic is used. The effectiveness of this heuristic
depends mainly on the scene itself. We can greatly improve it by using more
parameters but we will talk about it later.

We still need to implement the functions ``classify``, ``split_space`` and
``partition``. The last one is probably the most important. Where should we
split our space? Once again we're going to take the most simple solution for now.
We will use the spatial median splitting technique. At each depth of the tree,
the axis on which the division is made will be changed.

.. code:: rust

   impl<P: BoundingBox> KDtreeNode<P> {
       fn classify(items: &Items<P>, left_space: &AABB, right_space: &AABB) -> (Items<P>, Items<P>) {
           (
               // All items that overlap with the left space is taken
               items
                   .iter()
                   .filter(|item| left_space.intersect_box(&item.bb))
                   .cloned()
                   .collect(),
               // All items that overlap with the right space is taken
               items
                   .iter()
                   .filter(|item| right_space.intersect_box(&item.bb))
                   .cloned()
                   .collect(),
           )
       }

       fn split_space(space: &AABB, plan: &Plan) -> (AABB, AABB) {
           let mut left = space.clone();
           let mut right = space.clone();
           match plan {
               Plan::X(x) => {
                   left.1.x = *x;
                   right.0.x = *x
               }
               Plan::Y(y) => {
                   left.1.y = *y;
                   right.0.y = *y
               }
               Plan::Z(z) => {
                   left.1.z = *z;
                   right.0.z = *z;
               }
           }
           (left, right)
       }

       fn partition(space: &AABB, max_depth: usize) -> Plan {
           match max_depth % 3 {
               0 => Plan::X((space.0.x + space.1.x) / 2.),
               1 => Plan::Y((space.0.y + space.1.y) / 2.),
               _ => Plan::Z((space.0.z + space.1.z) / 2.),
           }
       }
   }

Intersect KD Tree
=================

Now that our we can build a ``KDtree``, we are able to compute our reduced list
of primitives that can intersect a ray.

Let's start with the ``KDtree`` struct:

.. code:: rust

   impl<P: BoundingBox> KDtree<P> {
       pub fn intersect(
           &self,
           ray_origin: &Vector3<f32>,
           ray_direction: &Vector3<f32>,
       ) -> Vec<Arc<P>> {
           // Check if the ray intersect the bounding box of the Kd Tree
           if self.space.intersect_ray(ray_origin, ray_direction) {
               self.root.intersect(ray_origin, ray_direction)
           } else {
               vec![]
           }
       }
   }

The same for ``KDtreeNode``:

.. code:: rust

   impl<P: BoundingBox> KDtreeNode<P> {
       pub fn intersect(
           &self,
           ray_origin: &Vector3<f32>,
           ray_direction: &Vector3<f32>,
       ) -> Vec<Arc<P>> {
           match self {
               // In case of leaf simply return the values
               Self::Leaf { values, .. } => values.clone(),
               // In case of an internal node check the sub-spaces
               Self::Node {
                   left_space,
                   left_node,
                   right_space,
                   right_node,
               } => {
                   let mut res = vec![];
                   if right_space.intersect_ray(ray_origin, ray_direction) {
                       // The ray intersect the left sub-space
                       res = right_node.intersect(ray_origin, ray_direction);
                   }
                   if left_space.intersect_ray(ray_origin, ray_direction) {
                       // The ray intersect the right sub-space
                       res.append(&mut left_node.intersect(ray_origin, ray_direction));
                   }
                   res
               }
           }
       }
   }

We are done with our naive implementation. It is obvious that a lot could be
done to improve the generated tree and we will explore this in the next part.
Still, this implementation brings a huge improvement to our rendering engine.

Surface Area Heuristic (SAH)
----------------------------

Theory
======

The SAH method provides both the ability to know which cutting plan is the best
and whether it is worth dividing the space (create a node) or not (create a sheet).
To do this, we need to calculate the *"cost"* of a leaf and the internal nodes for
each possible splitting plan.

Before explaining the method, we need to make a few assumptions:

- :math:`K_I`: The cost for primitive (triangle) **I**ntersection.
- :math:`K_T`: The cost for a **T**raversal step of the tree.

We can now calculate the cost of an intersection in our kd-tree. Let's say that,
for a given ray and kd-tree, the intersection function returns 13 primitives and
had to pass through 8 nodes of the tree.

:math:`C_{intersection} = 13 \times K_I + 8 \times K_T`.

It is fairly easy to calculate the cost of a leaf. It is simply the number of
primitives contained in the leaf :math:`|T|` multiplied by :math:`K_I`.

  :math:`C_{leaf} = |T| \times K_I`

It is somewhat more difficult to calculate the cost of an internal node given a
splitting plan. First we need to define more terms:

- :math:`p`: The splitting plan candidate.
- :math:`V`: The space of the whole node.
- :math:`|V_L|` and :math:`|V_R|`: The left and right space splitted by :math:`p`.
- :math:`|T_L|` and :math:`|T_R|`: The number of primitives that overlap the left
  and right volumes seperated by :math:`p`.
- :math:`SA(space)`: The function that calculate the surface area of a given space.
  This function is quite simple knowing the spaces are AABB, it's simply the
  multiplication of each side of the box.

The cost of an internal node is given by the following formula.

  :math:`C_{node}(p) = K_T + K_I \Big (|T_L| \times \frac{SA(V_L)}{SA(V)} + |T_R| \times \frac{SA(V_R)}{SA(V)} \Big)`

This formula may seem magical, but it is simply the cost of one traversal step
(:math:`K_T`), plus the expected cost of intersecting the two children. The
expected cost of intersecting a child is calculated by multiplying the number of
primitives in the child and the ratio of the area taken by the child's space.

Some shortcuts were made in the explanation of the formulas for more details take
a look at the `scientific reference paper
<http://www.irisa.fr/prive/kadi/Sujets_CTR/kadi/Kadi_sujet2_article_Kdtree.pdf>`_.

How to use SAH
==============

Sah gives us a way to compare splitting plans and select the best one. Once we
have it, Sah lets us know if it's worth cutting or if a leaf is preferable.

Basically what will change in our code is the partition function and the
termination function.

To divide our space, we are going to take all the possible splitting planes in
the 3 dimensions (called perfect splits). Then we will calculate the cost of the
partition and take the smallest one.

We need to define K_T and K_I in our implementation. For this the paper advice
to use:

- :math:`K_T=15`
- :math:`K_I=20`

Implementation of needed functions
==================================

These are the functions that use the above formulas to calculate the cost of a
split.

.. code:: rust

   static K_T: f32 = 15.;
   static K_I: f32 = 20.;

   impl<P: BoundingBox> KDtreeNode<P> {
     /// Compute surface area volume of a space (AABB).
     fn surface_area(v: &AABB) -> f32 {
         (v.1.x - v.0.x) * (v.1.y - v.0.y) * (v.1.z - v.0.z)
     }

     /// Surface Area Heuristic (SAH)
     fn cost(p: &Plan, v: &AABB, n_l: usize, n_r: usize) -> f32 {
         // Split space
         let (v_l, v_r) = Self::split_space(v, p);

         // Compute the surface area of both sub-space
         let (vol_l, vol_r) = (Self::surface_area(&v_l), Self::surface_area(&v_r));

         // Compute the surface area of the whole space
         let vol_v = vol_l + vol_r;

         // If one of the sub-space is empty then the split can't be worth
         if vol_v == 0. || vol_l == 0. || vol_r == 0. {
             return f32::INFINITY;
         }

         // Decrease cost if it cuts empty space
         let factor = if n_l == 0 || n_r == 0 { 0.8 } else { 1. };

         factor * (K_T + K_I * (n_l as f32 * vol_l / vol_v + n_r as f32 * vol_r / vol_v))
     }

     /// Return the perfect splits candidates of a given item and dimension.
     /// It's the clipped plans around the bounding box.
     fn perfect_splits(item: Arc<Item<P>>, v: &AABB, dim: usize) -> Vec<Plan> {
         let mut res = vec![];
         match dim {
             0 => {
                 res.push(Plan::X(item.bb.0.x.max(v.0.x)));
                 res.push(Plan::X(item.bb.1.x.min(v.1.x)));
             }
             1 => {
                 res.push(Plan::Y(item.bb.0.y.max(v.0.y)));
                 res.push(Plan::Y(item.bb.1.y.min(v.1.y)));
             }
             2 => {
                 res.push(Plan::Z(item.bb.0.z.max(v.0.z)));
                 res.push(Plan::Z(item.bb.1.z.min(v.1.z)));
             }
             _ => panic!("Invalid dimension number received: ({})", dim),
         }
         res
     }
   }

Build tree in :math:`O(N^2)`
============================

We can update the ``partition`` and ``new`` function in the simplest way.

.. code:: rust

   pub fn new(space: &AABB, mut items: Items<P>) -> Self {
       let (cost, plan) = Self::partition(&items, &space);

       // Check that the cost of the splitting is not higher than the cost of the leaf.
       if cost > K_I * items.len() as f32 {
           // Create the vector of primitives
           let mut values = vec![];
           while let Some(i) = items.pop() {
               values.push(i.value.clone());
           }
           return Self::Leaf {
               space: space.clone(),
               values,
           };
       }

       // Compute the new spaces divided by `plan`
       let (left_space, right_space) = Self::split_space(&space, &plan);

       // Compute which items are part of the left and right space
       let (left_items, right_items) = Self::classify(&items, &left_space, &right_space);

       Self::Node {
           left_node: Box::new(Self::new(&left_space, left_items)),
           right_node: Box::new(Self::new(&right_space, right_items)),
           left_space,
           right_space,
       }
   }

   /// Takes the items and space of a node and return the best splitting plan and his cost
   fn partition(items: &Items<P>, space: &AABB) -> (f32, Plan) {
       let (mut best_cost, mut best_plan) = (f32::INFINITY, Plan::X(0.));
       // For all the dimension
       for dim in 0..3 {
           for item in items {
               for plan in Self::perfect_splits(item.clone(), space, dim) {
                   // Compute the new spaces divided by `plan`
                   let (left_space, right_space) = Self::split_space(&space, &plan);
                   // Compute which items are part of the left and right space
                   let (left_items, right_items) =
                       Self::classify(&items, &left_space, &right_space);
                   // Compute the cost of the current plan
                   let cost = Self::cost(&plan, space, left_items.len(), right_items.len());
                   // If better update the best values
                   if cost < best_cost {
                       best_cost = cost;
                       best_plan = plan.clone();
                   }
               }
           }
       }
       (best_cost, best_plan)
   }

For each item, we use a ``classification`` function that also performs an
iteration on the items. This is why this partition implementation is in
:math:`O(N^2)`. As you can see in the `Benchmark`_ section, this is a problem
because the time saved by the sah method is lost in the construction of the kd-tree.

The full code is available `here <https://github.com/flomonster/kdtree-ray/tree/sah-quadratic>`_.

Build tree in :math:`O(N \log^2{N})`
====================================

Benchmark
---------

Render Runtime
==============

Runtime calculated using a raytracer and an image resolution of ``800x800``.

+------------+--------+------------------+-----------+---------+
| Model      | Nb Tri | No Kd-Tree (min) | Naive (s) | Sah (s) |
+============+========+==================+===========+=========+
| Armadillo  | 346k   | 50               | 38        | 10      |
+------------+--------+------------------+-----------+---------+
| Dragon     | 863k   | 115              | 65        | 10      |
+------------+--------+------------------+-----------+---------+
| Buddha     | 1m     | 150              | 63        | 10      |
+------------+--------+------------------+-----------+---------+
| ThaiStatue | 10m    | 1140             | 1980      |         |
+------------+--------+------------------+-----------+---------+

Build Tree Runtime
==================

+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Model      | Nb Tri | Naive (s) | :math:`O(N^2)` | :math:`O(N \log^2{N})` (s) | :math:`O(N \log N)` (s) |
+============+========+===========+================+============================+=========================+
| Armadillo  | 346k   | 0.352     | 28h            | 16                         |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Dragon     | 863k   | 0.853     | 178h           | 60                         |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Buddha     | 1m     | 1.016     | 240h           | 64                         |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| ThaiStatue | 10m    | 14.7      | 1000days       |                            |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
