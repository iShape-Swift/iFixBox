//
//  World.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat
import iShape

public struct World {
    
    public let freezeBoundary: FixBnd
    public let gravity: FixVec
    public let isDebug: Bool
    let collisionSolver: CollisionSolver
    
    
    var bodyStore: BodyStore
    //    public GridSpace LandGrid;
    
    public let posTimeStep: FixFloat
    public let positionIterations: Int
    public let velocityIterations: Int
    public let iTimeStep: FixFloat
    public let biasScale: FixFloat
    public var contacts: [Contact] = []
    public let impactStabilization: Int64
    
    public init(boundary: FixBnd, settings: WorldSettings, gravity: FixVec = FixVec(0, -10.fix), isDebug: Bool = false) {
        freezeBoundary = FixBnd(min: boundary.min - FixVec(settings.freezeMargin, settings.freezeMargin), max: boundary.max + FixVec(settings.freezeMargin, settings.freezeMargin))
        self.gravity = gravity
        self.isDebug = isDebug
        
        bodyStore = BodyStore(capacity: settings.bodyCapacity)
        //        LandGrid = new GridSpace(boundary, settings.GridSpaceFactor, allocator);
        
        posTimeStep = settings.posTimeStep
        positionIterations = settings.positionIterations
        velocityIterations = settings.velocityIterations
        iTimeStep = .unit.fixDiv(posTimeStep)
        biasScale = iTimeStep / Int64(settings.biasImpact)
        collisionSolver = CollisionSolver()
        impactStabilization = Int64(settings.impactStabilization)
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

    public mutating func add(body: Body) -> BodyHandler {
        var newBody = body
        newBody.boundary = self.boundary(shape: body.shape, transform: body.transform)
        let handler = bodyStore.add(body: newBody)

        // TODO update space grid

        return handler
    }
    
    public mutating func reset() {
        bodyStore.removeAll()
    }
    
    public func boundary(shape: Shape, transform: Transform) -> FixBnd {
        switch shape.form {
        case .circle:
            return FixBnd(radius: shape.radius, delta: transform.position)
        case .rect:
            return FixBnd(size: shape.size, transform: transform)
        case .polygon:
            return .zero
        }
    }
    
}
