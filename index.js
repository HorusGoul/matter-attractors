"use strict";

const Matter = require('matter-js');

/**
 * An attractors plugin for matter.js.
 * See the readme for usage and examples.
 * @module MatterAttractors
 */
const MatterAttractors = {
  // plugin meta
  name: 'matter-attractors', // PLUGIN_NAME
  version: '0.1.6', // PLUGIN_VERSION
  for: 'matter-js@^0.12.0',

  // installs the plugin where `base` is `Matter`
  // you should not need to call this directly.
  install: function(base) {
    base.after('Body.create', function() {
      MatterAttractors.Body.init(this);
    });

    base.before('Engine.update', function(engine) {
      MatterAttractors.Engine.update(engine);
    });
  },

  Body: {
    /**
     * Initialises the `body` to support attractors.
     * This is called automatically by the plugin.
     * @function MatterAttractors.Body.init
     * @param {Matter.Body} body The body to init.
     * @returns {void} No return value.
     */
    init: function(body) {
      body.plugin.attractors = body.plugin.attractors || [];
    }
  },

  Engine: {
    /**
     * Applies all attractors for all bodies in the `engine`.
     * This is called automatically by the plugin.
     * @function MatterAttractors.Engine.update
     * @param {Matter.Engine} engine The engine to update.
     * @returns {void} No return value.
     */
    update: function(engine) {
      let world = engine.world,
        bodies = Matter.Composite.allBodies(world);

      for (let i = 0; i < bodies.length; i += 1) {
        let bodyA = bodies[i],
          attractors = bodyA.plugin.attractors;

        if (attractors && attractors.length > 0) {
          for (let j = i + 1; j < bodies.length; j += 1) {
            let bodyB = bodies[j];

            for (let k = 0; k < attractors.length; k += 1) {
              let attractor = attractors[k],
                forceVector = attractor;

              if (Matter.Common.isFunction(attractor)) {
                forceVector = attractor(bodyA, bodyB);
              }
              
              if (forceVector) {
                Matter.Body.applyForce(bodyB, bodyB.position, forceVector);
              }
            }
          }
        }
      }
    }
  },
  
  /**
   * Defines some useful common attractor functions that can be used 
   * by pushing them to your body's `body.plugin.attractors` array.
   * @namespace MatterAttractors.Attractors
   * @property {number} gravityConstant The gravitational constant used by the gravity attractor.
   */
  Attractors: {
    gravityConstant: 0.001,

    /**
     * An attractor function that applies Newton's law of gravitation.
     * Use this by pushing `MatterAttractors.Attractors.gravity` to your body's `body.plugin.attractors` array.
     * The gravitational constant defaults to `0.001` which you can change 
     * at `MatterAttractors.Attractors.gravityConstant`.
     * @function MatterAttractors.Attractors.gravity
     * @param {Matter.Body} bodyA The first body.
     * @param {Matter.Body} bodyB The second body.
     * @returns {void} No return value.
     */
    gravity: function(bodyA, bodyB) {
      // use Newton's law of gravitation
      var bToA = Matter.Vector.sub(bodyB.position, bodyA.position),
        distanceSq = Matter.Vector.magnitudeSquared(bToA) || 0.0001,
        normal = Matter.Vector.normalise(bToA),
        magnitude = -MatterAttractors.Attractors.gravityConstant * (bodyA.mass * bodyB.mass / distanceSq),
        force = Matter.Vector.mult(normal, magnitude);

      // to apply forces to both bodies
      Matter.Body.applyForce(bodyA, bodyA.position, Matter.Vector.neg(force));
      Matter.Body.applyForce(bodyB, bodyB.position, force);
    },

    /**
     * An attractor function that applies magnetic pull to `metallic` bodies.
     * Use this by pushing `MatterAttractors.Attractors.magnet` to your body's `body.plugin.attractors` array.
     * The magnetic pull defaults to `0` (implying it's metallic but not magnetic) which you can change to a
     * positive number for a particular body by changing `body.plugin.magnet.pull`. Magnetic poles default to
     * the midpoint between the body's first two vertices and the midpoint between the body's middle and
     * middle-plus-one vertices. This can be changed by specifying two faces according to the body vertices'
     * indexes at `body.plugin.magnet.north` and `body.plugin.magnet.south`. If the magnetic pull is `0`,
     * magnetic poles are ignored for that body.
     * @function MatterAttractors.Attractors.magnet
     * @param {Matter.Body} bodyA The first body.
     * @param {Matter.Body} bodyB The second body.
     * @returns {void} No return value.
     */
    magnet: (function () {
      var northA = {cornerA: null, cornerB: null, x: 0, y: 0},
        southA = {cornerA: null, cornerB: null, x: 0, y: 0},
        northB = {cornerA: null, cornerB: null, x: 0, y: 0},
        southB = {cornerA: null, cornerB: null, x: 0, y: 0},
        emptyVector = {x: 0, y: 0},
        getVector = function (face, point) { // This returns a directional unit vector for a point according to where it's located relative to the face of a pole.
          var norm = Matter.Vector.perp(Matter.Vector.sub(face.cornerB, face.cornerA), true),
            rel = Matter.Vector.sub(point, face),
            targetNorm = null,
            halfFaceSq = Matter.Vector.magnitudeSquared(Matter.Vector.sub(face.cornerA, face));

          if (Matter.Vector.magnitudeSquared(rel) < halfFaceSq) {
            return Matter.Vector.neg(Matter.Vector.add(rel, norm));
          } else if (Matter.Vector.dot(norm, rel) <= 0) { // behind
            return norm;
          } else if (Matter.Vector.magnitudeSquared(Matter.Vector.sub(face.cornerA, point)) < halfFaceSq) { // near corner, need to pull towards face
            rel = Matter.Vector.sub(point, face.cornerA);
            return Matter.Vector.normalise(Matter.Vector.perp(rel, false));
          } else if (Matter.Vector.magnitudeSquared(Matter.Vector.sub(face.cornerB, point)) < halfFaceSq) { // use corner B
            rel = Matter.Vector.sub(point, face.cornerB);
            return Matter.Vector.normalise(Matter.Vector.perp(rel, true));
          } else { // farther away
            targetNorm = Matter.Vector.mult(Matter.Vector.normalise(norm), -0.5 * Matter.Vector.magnitude(rel) / Math.abs(Math.cos(Matter.Vector.angle(rel, norm))));
            return Matter.Vector.sub(targetNorm, rel);
          }
        },
        getForce = function (poleA, poleB, pull, distanceSq) {
          var direction = getVector(poleA, poleB),
            normal = Matter.Vector.normalise(direction),
            magnitude = -pull / Math.max(pull * 10, distanceSq);
            
          return Matter.Vector.mult(normal, magnitude);
        },
        applyForce = function (poleA, poleB, bodyA, bodyB, pullA, pullB) {
          var bToA = Matter.Vector.sub(poleB, poleA),
            distanceSq = Matter.Vector.magnitudeSquared(bToA),
            forceA = pullA ? Matter.Vector.neg(getForce(poleA, poleB, pullA, distanceSq)) : emptyVector,
            forceB = pullB ? getForce(poleB, poleA, pullB, distanceSq) : emptyVector,
            force = Matter.Vector.add(forceA, forceB);

          // to apply forces to both bodies
          Matter.Body.applyForce(bodyA, poleA, Matter.Vector.neg(force));
          Matter.Body.applyForce(bodyB, poleB, force);
        },
        getPole = function (index, vertices, pole) {
          var length = vertices.length,
            a = vertices[index % length],
            b = vertices[(index + 1) % length];

          pole.cornerA = a;
          pole.cornerB = b;
          pole.x = (a.x + b.x) / 2;
          pole.y = (a.y + b.y) / 2;
          return pole;
        },
        getSouthPole = function (magnet, vertices, pole) {
          if (typeof magnet.south !== 'number') {
            magnet.south = vertices.length / 2 >> 0;
          }

          return getPole(magnet.south, vertices, pole);
        },
        getNorthPole = function (magnet, vertices, pole) {
          if (typeof magnet.north !== 'number') {
            magnet.north = 0;
          }

          return getPole(magnet.north, vertices, pole);
        };

      return function (bodyA, bodyB) {
        var magnetA = bodyA.plugin.magnet,
          magnetB = bodyB.plugin.magnet,
          poleANorth = null,
          poleASouth = null,
          poleBNorth = null,
          poleBSouth = null,
          pullA = 0,
          pullB = 0,
          repel = -1,
          verticesA = bodyA.vertices,
          verticesB = bodyB.vertices;

        if (!magnetA) {
          magnetA = bodyA.plugin.magnet = {};
        }

        if (!magnetB || !(magnetA.pull || magnetB.pull)) {
          return;
        }

        pullA = magnetA.pull || 0;
        pullB = magnetB.pull || 0;
        if (pullA) {
          poleANorth = getNorthPole(magnetA, verticesA, northA);
          poleASouth = getSouthPole(magnetA, verticesA, southA);
        } else {
          poleANorth = poleASouth = bodyA.position;
          repel = 1;
        }
        if (pullB) {
          poleBNorth = getNorthPole(magnetB, verticesB, northB);
          poleBSouth = getSouthPole(magnetB, verticesB, southB);
        } else {
          poleBNorth = poleBSouth = bodyB.position;
          repel = 1;
        }
        
        applyForce(poleANorth, poleBNorth, bodyA, bodyB, repel * pullA, repel * pullB);
        applyForce(poleANorth, poleBSouth, bodyA, bodyB, pullA, pullB);
        applyForce(poleASouth, poleBNorth, bodyA, bodyB, pullA, pullB);
        applyForce(poleASouth, poleBSouth, bodyA, bodyB, repel * pullA, repel * pullB);
      };
    }())
  }
};

Matter.Plugin.register(MatterAttractors);

module.exports = MatterAttractors;

/**
 * @namespace Matter.Body
 * @see http://brm.io/matter-js/docs/classes/Body.html
 */

/**
 * This plugin adds a new property `body.plugin.attractors` to instances of `Matter.Body`.  
 * This is an array of callback functions that will be called automatically
 * for every pair of bodies, on every engine update.
 * @property {Function[]} body.plugin.attractors
 * @memberof Matter.Body
 */

/**
 * An attractor function calculates the force to be applied
 * to `bodyB`, it should either:
 * - return the force vector to be applied to `bodyB`
 * - or apply the force to the body(s) itself
 * @callback AttractorFunction
 * @param {Matter.Body} bodyA
 * @param {Matter.Body} bodyB
 * @returns {Vector|undefined} a force vector (optional)
 */