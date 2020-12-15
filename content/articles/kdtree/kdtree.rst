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

Many rendering engines are based on the physics of light rays (such as
`raytracer <https://en.wikipedia.org/wiki/Ray_tracing_(graphics)>`_).
This method allows to obtain realistic images, but is quite slow. Indeed, it
requires throwing a lot of rays and checking if they intersect an object in the scene.

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

It is important to note that if a primitive (e.g. triangle) is large enough and
intersects several cells (subspaces) then this primitive will be found in several
leaves of our kdtree.

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

Needed structure
================

Bounding Box
############

First of all, we have to define our AABB since that's what we're going to
manipulate.

.. code:: rust

   use cgmath::*;

   /// Axis-aligned bounding box is defined by two positions.
   #[derive(Clone, Debug)]
   pub struct AABB(pub Vector3<f32>, pub Vector3<f32>);

Some function will be needed, as described before:

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
           // Check that the ray intersects the square of the bounding box on
           // the X and Y axis.
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

           // Check that the ray intersects the square of the bounding box on
           // the Y and Z axis.

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
###############

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
our kdtree we have to return all primitives that could intersect the ray.
In other words we have to retrieve all the leaves intersecting the ray and return
their primitives. Since the same primitive could be stored in several leaves that
are intersected we'll have to use the **union** mathematical operation to merge
these primitives in one collection without doubles. This operation can only be
done fast using ``Set`` data structures. The only constraint to use a ``Set`` is
that ``Item`` will need to be hashable.

Plane
#####

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
####

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

Build kdtree
============

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

Let's now optimize the construction time of our kdtree. We noticed that the element
that makes our construction slow is the usage of the function ``classify``.

The reason for calling this function is to find out the number of items to the
left and right of a splitting candidate. To solve this problem we will use an
**incremental sweep** algorithm.

This algorithm needs to know if a splitting candidate is to the **left** or to
the **right** of its associated primitive.

In a given dimension, two counters are established:

- The number of primitives to the left of the candidate.
- The number of primitives to the right of the candidate.

These are the necessary information for the ``cost`` function. The algorithm will
then sweep the candidates in the order of their position and depending on whether
they are to the left or to the right of the primitive it will update its counters.

Here is a diagram to illustrate the steps of the algorithm.

.. figure:: /img/articles/kdtree/sweep.svg
   :alt: A 2D figure showing 3 primitives and their candidates
   :width: 70%

   2D figure of 3 primitives, green lines are for left candidates, red for right.


+----------------+-------+----------------+-----------------+
| Candidates     | Side  | Left count     | Right count     |
+================+=======+================+=================+
| Initialization | N/A   | 0              | 3               |
+----------------+-------+----------------+-----------------+
| 1              | Left  | 0 **+ 1** = 1  | 3               |
+----------------+-------+----------------+-----------------+
| 2              | Left  | 1 **+ 1** = 2  | 3               |
+----------------+-------+----------------+-----------------+
| 3              | Right | 2              | 3 **- 1** = 2   |
+----------------+-------+----------------+-----------------+
| 4              | Left  | 2 **+ 1** = 3  | 2               |
+----------------+-------+----------------+-----------------+
| 5              | Right | 3              | 2 **- 1** = 1   |
+----------------+-------+----------------+-----------------+
| 6              | Right | 3              | 1 **- 1** = 0   |
+----------------+-------+----------------+-----------------+

You may have noticed that the left counter has not exactly the right value. There
is an offset when the candidate is left. You will have to update the counter value
after calling the cost function.

The same kind of function can be used to find the items belonging to the left
and right subspace. But for this purpose the candidates must keep a reference on
their associated item.

Candidate
#########

A ``Candidate`` structure is needed to aggregate the separator planes, their side
(left/right) and a reference on the item.

.. code::rust

   #[derive(Debug)]
   pub struct Candidate<P: BoundingBox> {
       pub plane: Plane,
       pub is_left: bool,
       pub item: Arc<Item<P>>,
   }

We also need to be able to sort the candidates. For this we implement the trait
``Ord`` and ``Eq``.

.. code::rust

   impl Plane {
       /// To easily extract plane position
       pub fn value(&self) -> f32 {
           match self {
               Plane::X(v) => *v,
               Plane::Y(v) => *v,
               Plane::Z(v) => *v,
           }
       }
   }

   impl<P: BoundingBox> Ord for Candidate<P> {
       fn cmp(&self, other: &Self) -> Ordering {
           // Just need to compare the position of the plane
           if self.plane.value() < other.plane.value() {
               Ordering::Less
           } else {
               Ordering::Greater
           }
       }
   }

   // Required by Ord trait
   impl<P: BoundingBox> PartialOrd for Candidate<P> {
       fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
           Some(self.cmp(other))
       }
   }

   impl<P: BoundingBox> Eq for Candidate<P> {}

   // Required by Eq trait
   impl<P: BoundingBox> PartialEq for Candidate<P> {
       fn eq(&self, other: &Self) -> bool {
           self.plane.value() == other.plane.value()
       }
   }

Let's implement a function to generate these Candidate similar to the previous
function ``candidates``. We can also add other functions that will be usefull
for implentinIf the bounding box of the item is flat (so that its candidates have the same value), the left candidate must still appear first during the sweep.g the incremental sweep algorithm.

.. code::rust

   /// Candidates is a list of Candidate
   pub type Candidates<P> = Vec<Candidate<P>>;

   impl<P: BoundingBox> Candidate<P> {
     fn new(plane: Plane, is_left: bool, item: Arc<Item<P>>) -> Self {
         Candidate {
             plane,
             is_left,
             item,
         }
     }

     /// Return candidates (splits candidates) for a given dimension.
     pub fn gen_candidates(item: Arc<Item<P>>, dim: usize) -> Candidates<P> {
         match dim {
             0 => vec![
                 Candidate::new(Plane::X(item.bb.0.x), true, item.clone()),
                 Candidate::new(Plane::X(item.bb.1.x), false, item),
             ],
             1 => vec![
                 Candidate::new(Plane::Y(item.bb.0.y), true, item.clone()),
                 Candidate::new(Plane::Y(item.bb.1.y), false, item),
             ],
             2 => vec![
                 Candidate::new(Plane::Z(item.bb.0.z), true, item.clone()),
                 Candidate::new(Plane::Z(item.bb.1.z), false, item),
             ],
             _ => panic!("Invalid dimension number used: ({})", dim),
         }
     }

     /// Return the dimension value of the candidate
     pub fn dimension(&self) -> usize {
         match self.plane {
             Plane::X(_) => 0,
             Plane::Y(_) => 1,
             Plane::Z(_) => 2,
         }
     }

     pub fn is_left(&self) -> bool {
         self.is_left
     }

     pub fn is_right(&self) -> bool {
         !self.is_left
     }
   }

**Important**: The function ``gen_candidates`` returns first the left candidate
and then the right one. This detail is important. If the bounding box of
the item is flat (so that its candidates have the same value), the left candidate
must still appear first during the sweep.

Partition and Classify
######################

The ``partition`` function will have a lot of modification first instead of
returning a ``Plane`` we will return the sorted list of candidates and the index
of the best splitting candidate. Doing so will allow us to use an optimized
classify function.

.. code::rust

   /// Compute the best splitting candidate
   /// Return:
   /// * Cost of the split
   /// * The list of candidates (in the best dimension found)
   /// * Index of the best candidate
   fn partition(items: &Items<P>, space: &AABB) -> (f32, Candidates<P>, usize) {
       let mut best_cost = f32::INFINITY;
       let mut best_candidate_index = 0;
       let mut best_candidates = vec![];

       // For all the dimension
       for dim in 0..3 {
           // Generate candidates
           let mut candidates = vec![];
           for item in items {
               let mut c = Candidate::gen_candidates(item.clone(), dim);
               candidates.append(&mut c);
           }

           // Sort candidates
           candidates.sort_by(|a, b| a.cmp(&b));

           // Initialize counters
           let mut n_r = items.len();
           let mut n_l = 0;

           // Used to update best_candidates list
           let mut best_dim = false;

           // Find best candidate
           for (i, candidate) in candidates.iter().enumerate() {
               if candidate.is_right() {
                   n_r -= 1;
               }

               // Compute the cost of the current plane
               let cost = Self::cost(&candidate.plane, space, n_l, n_r);

               // If better update the best values
               if cost < best_cost {
                   best_cost = cost;
                   best_candidate_index = i;
                   best_dim = true;
               }

               if candidate.is_left() {
                   n_l += 1;
               }
           }

           // If a better candidate was found then keep the candidate list
           if best_dim {
               best_candidates = candidates;
           }
       }
       (best_cost, best_candidates, best_candidate_index)
   }

You must know that the sorting in Rust is stable that is to say in our case that
two candidates with the same plane will keep their order. This is important to
properly handle the case of flat bounding box. If you're using a non stable sort
you can slightly modify the comparison function of candidates to take into account
the ``is_left`` field.

The ``classify`` function is quite simple to implement.

.. code::rust

   fn classify(candidates: &Candidates<P>, best_index: usize)
     -> (Items<P>, Items<P>) {
       let mut left_items = Items::with_capacity(candidates.len() / 3);
       let mut right_items = Items::with_capacity(candidates.len() / 3);

       for i in 0..best_index {
           if candidates[i].is_left() {
               left_items.push(candidates[i].item.clone());
           }
       }
       for i in (1 + best_index)..candidates.len() {
           if candidates[i].is_right() {
               right_items.push(candidates[i].item.clone());
           }
       }
       (left_items, right_items)
   }

Finally we must adapt the function``KDtreeNode::new``.

.. code::rust

   pub fn new(space: &AABB, items: Items<P>) -> Self {
       let (cost, candidates, best_index) = Self::partition(&items, &space);

       // Check that the cost of the splitting is not higher than the cost of
       // the leaf.
       if cost > K_I * items.len() as f32 {
           return Self::Leaf {
               items: items.iter().cloned().collect(),
           };
       }

       // Compute the new spaces divided by `plane`
       let (left_space, right_space) =
           Self::split_space(&space, &candidates[best_index].plane);

       // Compute which items are part of the left and right space
       let (left_items, right_items) = Self::classify(&candidates, best_index);

       Self::Node {
           node: Box::new(InternalNode {
               left_node: Self::new(&left_space, left_items),
               right_node: Self::new(&right_space, right_items),
               left_space,
               right_space,
           }),
       }
   }

We now have a correct implementation of kdtree. However we can still speed up the
tree construction to be optimal. We will see how in the next part.

Build tree in :math:`O(N \log{N})`
====================================

This slows down our tree construction and the **sorting** of candidates. The idea
to optimize is to do one sort at the very beginning.

To do this we have to solve two problems:

- Take the sorting out of the inner loop of the ``partition`` function.
- Classify the candidates keeping them sorted.

The first problem can be fixed easily if we take as an argument a sorted list of
candidates (from all dimension) we can easily find the best candidate. We just
need more counters and be careful of candidates dimension.

We can modify our ``partition`` function:

.. code::rust

   /// Compute the best splitting candidate
   /// Return:
   /// * Cost of the split
   /// * Index of the best candidate
   /// * Number of items in the left partition
   /// * Number of items in the right partition
   fn partition(n: usize, space: &AABB, candidates: &Candidates<P>)
     -> (f32, usize, usize, usize) {
       let mut best_cost = f32::INFINITY;
       let mut best_candidate_index = 0;

       // Variables to keep count the number of items in both subspace for
       // each dimension
       let mut n_l = [0; 3];
       let mut n_r = [n; 3];

       // Keep n_l and n_r for the best splitting candidate
       let mut best_n_l = 0;
       let mut best_n_r = n;

       // Find best candidate
       for (i, candidate) in candidates.iter().enumerate() {
           let dim = candidate.dimension();

           // If the right candidate removes it from the right subspace
           if candidate.is_right() {
               n_r[dim] -= 1;
           }

           // Compute the cost of the split and update the best split
           let cost = Self::cost(&candidate.plane, space, n_l[dim], n_r[dim]);
           if cost < best_cost {
               best_cost = cost;
               best_candidate_index = i;
               best_n_l = n_l[dim];
               best_n_r = n_r[dim];
           }

           // If the left candidate add it from the left subspace
           if candidate.is_left() {
               n_l[dim] += 1;
           }
       }
       (best_cost, best_candidate_index, best_n_l, best_n_r)
   }

Now we need to split our candidate list given a splitting candidate. Not
forgetting to keep our new list sorted. We can do that in two steps:

- Determining which items is in the left/right/both subspace.
- Iterate on candidates adding them to the left/right list of candidates.

To mark items as on left/right/both subspace we can use a new **enum** ``Side``
and items id field.

.. code::rust

   /// Useful to classify candidates
   #[derive(Debug, Clone)]
   pub enum Side { Left, Right, Both }

Instead of instantiating a list of ``Side`` each time we call the classify function.
We can create this list once at the beginning and pass it through the recursive
calls of our tree.

Let's implement our new ``classify`` function:

.. code::rust

    fn classify(
        candidates: Candidates<P>,
        best_index: usize,
        sides: &mut Vec<Side>,
    ) -> (Candidates<P>, Candidates<P>) {
        // Step 1: Udate sides to classify items
        Self::classify_items(&candidates, best_index, sides);

        // Step 2: Splicing candidates left and right subspace
        Self::splicing_candidates(candidates, &sides)
    }

    /// Step 1 of classify.
    /// Given a candidate list and a splitting candidate identify wich items are
    /// part of the left, right and both subspaces.
    fn classify_items(
        candidates: &Candidates<P>,
        best_index: usize,
        sides: &mut Vec<Side>
    ) {
        let best_dimension = candidates[best_index].dimension();
        for i in 0..(best_index + 1) {
            if candidates[i].dimension() == best_dimension {
                if candidates[i].is_right() {
                    sides[candidates[i].item.id] = Side::Left;
                } else {
                    sides[candidates[i].item.id] = Side::Both;
                }
            }
        }
        for i in best_index..candidates.len() {
            if candidates[i].dimension() == best_dimension
               && candidates[i].is_left() {
                sides[candidates[i].item.id] = Side::Right;
            }
        }
    }

    // Step 2: Splicing candidates left and right subspace given items sides
    fn splicing_candidates(
        mut candidates: Candidates<P>,
        sides: &Vec<Side>,
    ) -> (Candidates<P>, Candidates<P>) {
        let estimated_size = candidates.len() / 2;
        let mut left_candidates = Candidates::with_capacity(estimated_size);
        let mut right_candidates = Candidates::with_capacity(estimated_size);

        for e in candidates.drain(..) {
            match sides[e.item.id] {
                Side::Left => left_candidates.push(e),
                Side::Right => right_candidates.push(e),
                Side::Both => {
                    right_candidates.push(e.clone());
                    left_candidates.push(e);
                }
            }
        }
        (left_candidates, right_candidates)
    }

Let's adapt the function``KDtreeNode::new``.

.. code::rust

   pub fn new(
       space: &AABB,
       mut candidates: Candidates<P>,
       n: usize, // The number of items
       sides: &mut Vec<Side>,
   ) -> Self {
       let (cost, best_index, n_l, n_r) =
           Self::partition(n, &space, &candidates);

       // Check that the cost of the splitting is not higher than the cost of
       // the leaf.
       if cost > K_I * n as f32 {
           // Create the set of primitives
           let mut items = HashSet::with_capacity(n);
           candidates
               .drain(..)
               .filter(|e| e.is_left() && e.dimension() == 0)
               .for_each(|e| {
                   items.insert(e.item);
               });
           return Self::Leaf { items };
       }

       // Compute the new spaces divided by `plane`
       let (left_space, right_space) =
           Self::split_space(&space, &candidates[best_index].plane);

       // Compute which candidates are part of the left and right space
       let (left_candidates, right_candidates) =
           Self::classify(candidates, best_index, sides);

       Self::Node {
           node: Box::new(InternalNode {
             left_node: Self::new(&left_space, left_candidates, n_l, sides),
             right_node: Self::new(&right_space, right_candidates, n_r, sides),
             left_space,
             right_space,
           }),
       }
   }

Since we are mixing up candidates with different dimensions, can simplify
``gen_candidates`` function that doesn't need a dimension anymore.

.. code::rust

   impl<P: BoundingBox> Candidate<P> {
     /// Return candidates (splits candidates) for all dimension.
     pub fn gen_candidates(item: Arc<Item<P>>, bb: &AABB) -> Candidates<P> {
         vec![
             Candidate::new(Plane::X(bb.0.x), true, item.clone()),
             Candidate::new(Plane::Y(bb.0.y), true, item.clone()),
             Candidate::new(Plane::Z(bb.0.z), true, item.clone()),
             Candidate::new(Plane::X(bb.1.x), false, item.clone()),
             Candidate::new(Plane::Y(bb.1.y), false, item.clone()),
             Candidate::new(Plane::Z(bb.1.z), false, item),
         ]
     }
   }

We can also simplify ``Item`` since it doesn't need a bounding box as field
anymore.

.. code::rust

   #[derive(Debug)]
   pub struct Item<P: BoundingBox> {
       pub value: Arc<P>,
       pub id: usize,
   }

   impl<P: BoundingBox> Item<P> {
       pub fn new(value: P, id: usize) -> Self {
           Item {
               value: Arc::new(value),
               id,
           }
       }
   }

Finally we must create the initial sorted list of candidate and the list of sides.
All of that will be done in the ```KDtree::new`` function:

.. code::rust

   pub fn new(mut values: Vec<P>) -> Self {
       let mut space = AABB(Vector3::<f32>::max_value(),
                            Vector3::<f32>::min_value());
       let n = values.len();
       let mut candidates = Candidates::with_capacity(n * 6);
       for (id, v) in values.drain(..).enumerate() {
           // Create items from values
           let bb = v.bounding_box();
           let item = Arc::new(Item::new(v, id));
           candidates.append(&mut Candidate::gen_candidates(item, &bb));

           // Update space with the bounding box of the item
           space.0.x = space.0.x.min(bb.0.x);
           space.0.y = space.0.y.min(bb.0.y);
           space.0.z = space.0.z.min(bb.0.z);
           space.1.x = space.1.x.max(bb.1.x);
           space.1.y = space.1.y.max(bb.1.y);
           space.1.z = space.1.z.max(bb.1.z);
       }

       // Sort candidates only once at the begining
       candidates.sort_by(|a, b| a.cmp(&b));

       // Will be used to classify candidates
       let mut sides = vec![Side::Both; n];
       let root = KDtreeNode::new(&space, candidates, n, &mut sides);
       KDtree { space, root }
   }

We're done with our final implementation! Don't forget that the complete code
of each version is available.

Benchmark
---------

Render Runtime
==============

Runtime calculated using a raytracer and an image resolution of ``800x800``.

+------------+--------+----------------+-----------+---------+
| Model      | Nb Tri | No Kd-Tree (s) | Naive (s) | Sah (s) |
+============+========+================+===========+=========+
| Armadillo  | 346k   | 3,000          | 115       | 1       |
+------------+--------+----------------+-----------+---------+
| Dragon     | 863k   | 6,900          | 293       | 10      |
+------------+--------+----------------+-----------+---------+
| Buddha     | 1m     | 9,000          | 292       | 14      |
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
| Armadillo  | 346k   | 0.352     | 28h            | 8                          | 4                       |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Dragon     | 863k   | 0.853     | 178h           | 30                         | 14                      |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| Buddha     | 1m     | 1.016     | 240h           | 31                         | 17                      |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
| ThaiStatue | 10m    | 14.7      | 1,000days      | 500                        | 245                     |
+------------+--------+-----------+----------------+----------------------------+-------------------------+
