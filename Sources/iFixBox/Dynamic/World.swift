//
//  World.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct World {
    
    public let freezeBoundary: Boundary
    public let gravity: FixVec
    let collisionSolver: CollisionSolver
    
    var bodyStore: BodyStore
    //    public GridSpace LandGrid;
    
    public let timeStep: FixFloat
    
    public init(boundary: Boundary, settings: WorldSettings, gravity: FixVec) {
        freezeBoundary = Boundary(min: boundary.min - FixVec(settings.freezeMargin, settings.freezeMargin), max: boundary.max + FixVec(settings.freezeMargin, settings.freezeMargin))
        self.gravity = gravity
        
        bodyStore = BodyStore(capacity: settings.bodyCapacity)
        //        LandGrid = new GridSpace(boundary, settings.GridSpaceFactor, allocator);
        
        timeStep = settings.timeStep
        collisionSolver = CollisionSolver()
    }
 
    public func actor(handler: BodyHandler) -> Actor {
        bodyStore.actor(handler: handler)
    }
    
    public func actor(id: Int64) -> Actor {
        bodyStore.actor(handler: BodyHandler(id: id))
    }

    public mutating func set(actor: Actor) -> BodyHandler {
        bodyStore.set(actor: actor)
    }

    public mutating func add(body: Body) -> BodyHandler  {
        let handler = bodyStore.add(body: body)

        // TODO update space grid

        return handler
    }
    
    public mutating func reset() {
        bodyStore.removeAll()
    }
    
}
