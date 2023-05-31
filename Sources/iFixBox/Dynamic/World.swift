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
    
    public init(boundary: Boundary, settings: WorldSettings, gravity: FixVec = FixVec(0, -10.fix), isDebug: Bool = false) {
        freezeBoundary = Boundary(min: boundary.min - FixVec(settings.freezeMargin, settings.freezeMargin), max: boundary.max + FixVec(settings.freezeMargin, settings.freezeMargin))
        self.gravity = gravity
        self.isDebug = isDebug
        
        bodyStore = BodyStore(capacity: settings.bodyCapacity)
        //        LandGrid = new GridSpace(boundary, settings.GridSpaceFactor, allocator);
        
        posTimeStep = settings.posTimeStep
        positionIterations = settings.positionIterations
        velocityIterations = settings.velocityIterations
        iTimeStep = .unit.div(posTimeStep)
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
    
    public func boundary(shape: Shape, transform: Transform) -> Boundary {
        switch shape.form {
        case .circle:
            return Boundary(radius: shape.radius, delta: transform.position)
        case .rect:
            return Boundary(size: shape.size, transform: transform)
        case .polygon:
            return .zero
        }
    }
    
}
