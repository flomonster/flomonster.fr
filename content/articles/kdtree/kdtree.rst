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

* `Naive implementation <https://github.com/flomonster/kdtree-ray/tree/naive>`_

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

We can also define ``Items`` which is a list of ``Item``.

.. code:: rust

   pub type Items<P> = Vec<Item<P>>;

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
               let item = Item::new(v);

               // Update space with the bounding box of the item
               space.0.x = space.0.x.min(item.bb.0.x);
               space.0.y = space.0.y.min(item.bb.0.y);
               space.0.z = space.0.z.min(item.bb.0.z);
               space.1.x = space.1.x.max(item.bb.1.x);
               space.1.y = space.1.y.max(item.bb.1.y);
               space.1.z = space.1.z.max(item.bb.1.z);

               items.push(item);
           }

           // Create the root KDtreeNode
           // We provide the computed space, items and a max_depth of our tree
           let root = KDtreeNode::new(&space, items, 20);
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
                   values.push(i.value);
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

Intersect KDtree
================

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

Benchmark & Conclusion
======================

.. TODO
