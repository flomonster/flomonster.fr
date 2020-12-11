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

This article describes the implementation of several versions of kdtree starting
from a naive version to an optimized version. Each version of the code is available:

* Naive implementation: `<https://github.com/flomonster/kdtree-ray/tree/naive>`_.
* Sah in :math:`O(N^2)`: `<https://github.com/flomonster/kdtree-ray/tree/sah-quadratic>`_.
* Sah in :math:`O(N \log^2{N})`: `<https://github.com/flomonster/kdtree-ray/tree/sah-log2>`_.
* Sah in :math:`O(N \log{N})`: `<https://github.com/flomonster/kdtree-ray/tree/sah-log>`_.

The models used for benchmark are from the `Stanford Scanning Repository
<http://graphics.stanford.edu/data/3Dscanrep/>`_.

General principles
------------------

Context
=======

Many rendering engines are based on the physics of light rays. This method allows
to obtain realistic images, but is quite slow. Indeed, it requires throwing a
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
primitives overlap the new subspaces. For an optimal kdtree, we must
divide the space optimally and stop recursion optimally.

Naive implementation
--------------------

This version will serve as a proof of concept. And yet, it will significantly
reduce the intersection search algorithm runtime.

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
   pub struct InternalNode<P: BoundingBox> {
       left_space: AABB,
       left_node: KDtreeNode<P>,
       right_space: AABB,
       right_node: KDtreeNode<P>,
   }

   #[derive(Clone, Debug)]
   pub enum KDtreeNode<P: BoundingBox> {
       Leaf { items: HashSet<Arc<Item<P>>> },
       Node { node: Box<InternalNode<P>> },
   }

The implementation of this structure is really important. We need to optimize the memory used by the tree.

- A primitive could be in several branches of our tree. To avoid copies, we use
  ``Arc`` which keeps only one reference on the objects.
- Then, dividing the structure in two using ``InternalNode`` reduces the size of
  ``KDtreeNode`` from ``72`` to ``56`` bytes. This doesn't change anything for our
  internal nodes since they need an instance of ``InternalNode``, but our leaves
  are much lighter.

Note that our leaves stores ``Items<P>`` and not ``P`` we'll talk about ``Item``
later. What we can explain now is the data structure used to store these items.
We're using an ``HashSet`` instead of a ``Vec``. When we are intersecting a ray to
our kdtree we have to return all candidates primitives that could intersect the ray.
In other words we have to retrieve all the leaves intersecting the ray and return
their primitives. So we'll have to use the **union** mathematical operation to merge
these primitives in one collection without doubles. This operation can only be
done using ``Set`` data structures. In addition our ``Item`` will need to be hashable.

Plane
=====

Let's create a structure that represents a split in a space. Since our space is
in 3D a plane is perfect to represents this seperation.

.. code:: rust

   #[derive(Clone, Debug)]
   pub enum Plane {
       X(f32), // Split on the X-axis
       Y(f32), // Split on the Y-axis
       Z(f32), // Split on the Z-axis
   }


Item
====

Before starting the kdtree implementation we need to define and explain Items.
``Item`` structure will allow us two things:

- First simplify the code by aggregate a primitive and his bounding box.
- Then being hashable needed by ``HashSet`` (into our leaves).
  To do so an ``id`` will be added in the structure.

.. code:: rust

   use std::hash::{Hash, Hasher};
   use std::sync::Arc;

   #[derive(Debug)]
   pub struct Item<P: BoundingBox> {
       pub value: Arc<P>,
       pub bb: AABB,
       pub id: usize,
   }

   impl<P: BoundingBox> Item<P> {
       pub fn new(value: P, bb: AABB, id: usize) -> Self {
           Item {
               value: Arc::new(value),
               bb,
               id,
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
               id: self.id,
           }
       }
   }

   /// Implementation of the Hash trait
   impl<P: BoundingBox> Hash for Item<P> {
       fn hash<H: Hasher>(&self, state: &mut H) {
           self.id.hash(state);
       }
   }

   impl<P: BoundingBox> Eq for Item<P> {}
   impl<P: BoundingBox> PartialEq for Item<P> {
       fn eq(&self, other: &Self) -> bool {
           self.id == other.id
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
           let mut space =
               AABB(Vector3::<f32>::max_value(), Vector3::<f32>::min_value());
           let mut items = Items::with_capacity(values.len());
           // Enumerate the values to get a tuple (id, value)
           for (id, v) in values.drain(..).enumerate() {
               // Create items from values
               let bb = v.bounding_box();
               items.push(Arc::new(Item::new(v, bb.clone(), id)));

               // Update space with the bounding box of the item
               space.0.x = space.0.x.min(bb.0.x);
               space.0.y = space.0.y.min(bb.0.y);
               space.0.z = space.0.z.min(bb.0.z);
               space.1.x = space.1.x.max(bb.1.x);
               space.1.y = space.1.y.max(bb.1.y);
               space.1.z = space.1.z.max(bb.1.z);
           }
           // Create the root of the kdtree with a maximum depth of 10
           let root = KDtreeNode::new(&space, items, 10);
           KDtree { space, root }
       }
   }


Note that the **maximum depth** will allow us to create a stopping criterion easily.
The value was chosen arbitrarily.

KDtreeNode
##########

Let's implement the function to create a ``KDtreeNode``.

.. code:: rust

   impl<P: BoundingBox> KDtreeNode<P> {
     pub fn new(space: &AABB, items: Items<P>, max_depth: usize) -> Self {
        // Heuristic to terminate the recursion
        if items.len() <= 15 || max_depth == 0 {
            return Self::Leaf {
                items: items.iter().cloned().collect(),
            };
        }

        // Find a plane to partition the space
        let plane = Self::partition(&space, max_depth);

        // Compute the new spaces divided by `plane`
        let (left_space, right_space) = Self::split_space(&space, &plane);

        // Compute which items are part of the left and right space
        let (left_items, right_items) =
            Self::classify(&items, &left_space, &right_space);

        Self::Node {
            node: Box::new(InternalNode {
                left_node: Self::new(&left_space, left_items, max_depth - 1),
                right_node: Self::new(&right_space, right_items, max_depth - 1),
                left_space,
                right_space,
            }),
        }
      }
   }

There is a lot going on here. This contains the basic algorithm to build our kdtree.
Note that an arbitrary heuristic is used. The effectiveness of this heuristic
depends mainly on the scene itself. We can greatly improve it by using more
parameters but we will talk about it later.

We still need to implement the functions ``classify``, ``split_space`` and
``partition``. This last function is probably the most important since, depending
on where we split our space, the kdtree will be more or less efficient.
Once again we're going to take the most simple solution for now.
We will use the spatial **median splitting technique**. At each depth of the tree,
the axis on which the division is made will be changed.

.. code:: rust

   impl<P: BoundingBox> KDtreeNode<P> {
     fn classify(items: &Items<P>, left_space: &AABB, right_space: &AABB)
       -> (Items<P>, Items<P>) {
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

     fn split_space(space: &AABB, plane: &Plane) -> (AABB, AABB) {
         let mut left = space.clone();
         let mut right = space.clone();
         match plane {
             Plane::X(x) => {
                 left.1.x = x.max(space.0.x).min(space.1.x);
                 right.0.x = x.max(space.0.x).min(space.1.x);
             }
             Plane::Y(y) => {
                 left.1.y = y.max(space.0.y).min(space.1.y);
                 right.0.y = y.max(space.0.y).min(space.1.y);
             }
             Plane::Z(z) => {
                 left.1.z = z.max(space.0.z).min(space.1.z);
                 right.0.z = z.max(space.0.z).min(space.1.z);
             }
         }
         (left, right)
     }

     fn partition(space: &AABB, max_depth: usize) -> Plane {
         match max_depth % 3 {
             0 => Plane::X((space.0.x + space.1.x) / 2.),
             1 => Plane::Y((space.0.y + space.1.y) / 2.),
             _ => Plane::Z((space.0.z + space.1.z) / 2.),
         }
     }
   }

You may have noticed that the ``perfect_splits`` function clips the plane to the
space ``v``. This is perfectly useless for the naive version. The median plane will
never be outside the space ``v``. However later versions might call the function
with a plane that is not contained in ``v``.

Intersect KD Tree
=================

Now that our kdtree is built, we are able to compute our reduced list of primitives
that can intersect a ray.

Let's implement this function starting with the ``KDtree`` struct:

.. code:: rust

   impl<P: BoundingBox> KDtree<P> {
     /// This function takes a ray and return a reduced list of candidates that
     /// can be intersected by the ray.
     pub fn intersect(
         &self,
         ray_origin: &Vector3<f32>,
         ray_direction: &Vector3<f32>,
     ) -> Vec<Arc<P>> {
         // Check if the ray intersect the bounding box of the Kd Tree
         if self.space.intersect_ray(ray_origin, ray_direction) {
             // Create an empty set of items
             let mut items = HashSet::new();
             // This call will fill our set of primitives
             self.root.intersect(ray_origin, ray_direction, &mut items);
             // Convert the set of items in vector of primitives
             items.iter().map(|e| e.value.clone()).collect()
         } else {
             // If the ray doesn't intersect the global bounding box no
             // primitives can be intersected
             vec![]
         }
     }
   }

The ``KDtreeNode::intersect`` is responsible to walk through the kdtree and
when necessary fill the given set ``intersected_items``.

.. code:: rust

   impl<P: BoundingBox> KDtreeNode<P> {
     pub fn intersect(
         &self,
         ray_origin: &Vector3<f32>,
         ray_direction: &Vector3<f32>,
         intersected_items: &mut HashSet<Arc<Item<P>>>,
     ) {
         match self {
             Self::Leaf { items } => {
                 // The ray intersect a leaf so we his items to the set.
                 intersected_items.extend(items.clone());
             }
             Self::Node { node } => {
                 if node.right_space.intersect_ray(ray_origin, ray_direction) {
                   node.right_node
                       .intersect(ray_origin, ray_direction, intersected_items);
                 }
                 if node.left_space.intersect_ray(ray_origin, ray_direction) {
                   node.left_node
                       .intersect(ray_origin, ray_direction, intersected_items);
                 }
             }
         }
     }
   }

Tips and analysis
=================

We are done with our naive implementation. It is obvious that a lot could be
done to improve the generated tree and we will explore this in the next part.
Still, this implementation brings a huge improvement to our rendering engine.

One way to use a kdtrees for your scenes is to store each model in a kdtree and
then you can store your kdtrees (of models) in a global kdtree for the entire scene.

To be able to create a kdtree of kdtree you only need to implement the trait
``BoundingBox`` for the ``KDtree`` struct.

.. code:: rust

   impl<P: BoundingBox> BoundingBox for KDtree<P>
   {
       fn bounding_box(&self) -> AABB {
           self.space.clone()
       }
   }

A simple trick that allows you to render scenes with a large number of models
and primitives.

Surface Area Heuristic (SAH)
----------------------------

Theory
======

The SAH method provides both the ability to know which cutting plane is the best
and whether it is worth dividing the space (create a node) or not (create a sheet).
To do this, we need to calculate the *"cost"* of a leaf and the internal nodes for
each possible splitting plane.

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
splitting plane. First we need to define more terms:

- :math:`p`: The splitting plane candidate.
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

Sah gives us a way to compare splitting planes and select the best one. Once we
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
     fn cost(p: &Plane, v: &AABB, n_l: usize, n_r: usize) -> f32 {
         // Split space
         let (v_l, v_r) = Self::split_space(v, p);

         // Compute the surface area of both subspace
         let vol_l = Self::surface_area(&v_l);
         let vol_r = Self::surface_area(&v_r);

         // Compute the surface area of the whole space
         let vol_v = vol_l + vol_r;

         // If one of the subspace is empty then the split can't be worth
         if vol_v == 0. || vol_l == 0. || vol_r == 0. {
             return f32::INFINITY;
         }

         // Decrease cost if it cuts empty space
         let factor = if n_l == 0 || n_r == 0 { 0.8 } else { 1. };

         // Node cost formula
         factor * (K_T + K_I * (n_l as f32 * vol_l / vol_v +
                                n_r as f32 * vol_r / vol_v))
     }
   }

The cost formula is slightly different from the one presented above. A factor of
``0.8`` has been added in case one of the subspaces does not contain any items.
This small change improves the results somewhat.

Generate candidates
===================

We are able to evaluate the cost of a split. However, there remains a problem,
in a given space there are an infinite number of planes of partition. It is
therefore necessary to choose an arbitrary number of planes that we will compare
with each other and select the one with the lowest cost. These planes will be
called candidate.

We can observe that in a given dimension two different planes that separate the
elements in the same way will have a very close cost. This being said we can
choose as candidates the planes formed by the sides of the bounding boxes of
each primitive.

.. figure:: /img/articles/kdtree/candidates.svg
   :alt: A 2D figure of splitting candidates.
   :width: 70%

   An example of splitting candidates in 2D. The green lines are splitting
   candidates in a dimension, the red in another.

Given an item and a dimension we need to be able to generate such splitting candidates.

.. code:: rust

   impl<P: BoundingBox> Item<P> {
       pub fn candidates(&self, dim: usize) -> Vec<Plane> {
           match dim {
               0 => vec![Plane::X(self.bb.0.x), Plane::X(self.bb.1.x)],
               1 => vec![Plane::Y(self.bb.0.y), Plane::Y(self.bb.1.y)],
               2 => vec![Plane::Z(self.bb.0.z), Plane::Z(self.bb.1.z)],
               _ => panic!("Invalid dimension number received: ({})", dim),
           }
       }
   }

Note that we generate planes that are not mandatory within a space. The clipping
of the ``split_space`` function is needed.

Build tree in :math:`O(N^2)`
============================

We can update the ``partition`` and ``new`` functions to get rid of our heuristics
and use the sah instead (no more ``max_depth``). This modification will greatly
increase the construction time of the kdtree. We will ignore this for now.

.. code:: rust

   pub fn new(space: &AABB, items: Items<P>) -> Self {
       let (cost, plane) = Self::partition(&items, &space);

       // Check that the cost of the splitting is not higher than the cost of
       // the leaf.
       if cost > K_I * items.len() as f32 {
           return Self::Leaf {
               items: items.iter().cloned().collect(),
           };
       }

       // Compute the new spaces divided by `plane`
       let (left_space, right_space) = Self::split_space(&space, &plane);

       // Compute which items are part of the left and right space
       let (left_items, right_items) =
           Self::classify(&items, &left_space, &right_space);

       Self::Node {
           node: Box::new(InternalNode {
               left_node: Self::new(&left_space, left_items),
               right_node: Self::new(&right_space, right_items),
               left_space,
               right_space,
           }),
       }
   }

   /// Takes the items and space of a node and return the best splitting plane
   /// and his cost
   fn partition(items: &Items<P>, space: &AABB) -> (f32, Plane) {
       let (mut best_cost, mut best_plane) = (f32::INFINITY, Plane::X(0.));
       // For all the dimension
       for dim in 0..3 {
           for item in items {
               for plane in item.candidates(dim) {
                   // Compute the new spaces divided by `plane`
                   let (left_space, right_space) =
                       Self::split_space(&space, &plane);

                   // Compute which items are part of the left and right space
                   let (left_items, right_items) =
                       Self::classify(&items, &left_space, &right_space);

                   // Compute the cost of the current plane
                   let cost = Self::cost(&plane, space,
                                         left_items.len(), right_items.len());

                   // If better update the best values
                   if cost < best_cost {
                       best_cost = cost;
                       best_plane = plane.clone();
                   }
               }
           }
       }
       (best_cost, best_plane)
   }


For each **candidate**, we call ``classify`` function that performs an iteration
on all items. This is why this partition implementation is in :math:`O(N^2)`.
As you can check in the `Benchmark`_ section, this implementation is not viable.

Build tree in :math:`O(N \log^2{N})`
====================================

Build tree in :math:`O(N \log{N})`
====================================

Benchmark
---------

Render Runtime
==============

Runtime calculated using a raytracer and an image resolution of ``800x800``.

+------------+--------+----------------+-----------+---------+
| Model      | Nb Tri | No Kd-Tree (s) | Naive (s) | Sah (s) |
+============+========+================+===========+=========+
| Armadillo  | 346k   | 3,000          | 38        | 10      |
+------------+--------+----------------+-----------+---------+
| Dragon     | 863k   | 6,900          | 65        | 10      |
+------------+--------+----------------+-----------+---------+
| Buddha     | 1m     | 9,000          | 63        | 10      |
+------------+--------+----------------+-----------+---------+
| ThaiStatue | 10m    | 68,400         | 1,980     | 95      |
+------------+--------+----------------+-----------+---------+

The naive implementation is not optimized at all. We can expect to get better
results with a tweaked implementation.

Tree construction runtime
=========================

+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Model      | Nb Tri | Naive (s) | :math:`O(N^2)` | :math:`O(N \log^2{N})` (s) | :math:`O(N \log N)` (s) |
+============+========+===========+================+============================+=========================+
| Armadillo  | 346k   | 0.352     | 28h            | 16                         |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Dragon     | 863k   | 0.853     | 178h           | 60                         |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Buddha     | 1m     | 1.016     | 240h           | 64                         |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| ThaiStatue | 10m    | 14.7      | 1,000days      | 1,918 (need confirm)       |                         |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
