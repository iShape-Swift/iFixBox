//
//  Body.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct Body {
    
    static let empty = Body(id: -1, transform: .zero, isDynamic: false, material: .ordinary)
    
    public let id: Int64

    public private (set) var mass: FixFloat
    private (set) var invMass: FixFloat
    private (set) var unitInertia: FixFloat
    private (set) var inertia: FixFloat
    private (set) var invInertia: FixDouble

    public private (set) var shape: Shape
    public private (set) var material: Material
    public private (set) var velocity: Velocity
    public private (set) var acceleration: Acceleration
    public private (set) var transform: Transform
    
    public var boundary: Boundary
    public let applyGravity: Bool
    public var isAlive: Bool
    public let isDynamic: Bool

    public init(id: Int64, transform: Transform, isDynamic: Bool = true, material: Material = .ordinary, applyGravity: Bool = true) {
        self.id = id
        self.isDynamic = isDynamic

        self.material = material
        self.shape = Shape.empty
        self.invMass = 0
        self.mass = 0
        self.inertia = 0
        self.unitInertia = 0
        self.invInertia = 0
        self.velocity = Velocity.zero
        self.transform = transform
        self.boundary = Boundary.zero
        self.isAlive = true
        self.applyGravity = applyGravity && isDynamic
        self.acceleration = Acceleration.zero
    }

    public mutating func attach(shape: Shape) {
        self.shape = shape
        if isDynamic {
            mass = shape.area.mul(material.density)
            invMass = .unit.div(mass)
            unitInertia = shape.unitInertia
            inertia = unitInertia.mul(mass)
            invInertia = .unit.div(fixDouble: inertia)
        }
        self.boundary = .zero
    }

    public mutating func addForce(force: FixVec, point: FixVec) {
        guard point != transform.position else {
            self.addAccelerationToCenterOfMass(force * invMass)
            return
        }

        let r = point - transform.position
        let n = r.normalize
        let projF = force.dotProduct(n)
        let a = projF.mul(invMass) * n
        let moment = force.crossProduct(r)
        let wa = moment.mul(fixDouble: invInertia)

        acceleration = Acceleration(linear: acceleration.linear + a, angular: acceleration.angular + wa)
    }

    public mutating func addAcceleration(acceleration: FixVec, point: FixVec) {
        guard point != transform.position else {
            self.addAccelerationToCenterOfMass(acceleration)
            return
        }

        let r = point - transform.position
        let n = r.normalize
        let a = acceleration.dotProduct(n) * n
        let wa = acceleration.crossProduct(r).mul(unitInertia)

        self.acceleration = Acceleration(linear: self.acceleration.linear + a, angular: self.acceleration.angular + wa)
    }

    public mutating func addVelocity(velocity: FixVec, point: FixVec) {
        guard point != transform.position else {
            self.addVelocityToCenterOfMass(velocity)
            return
        }

        let r = point - transform.position
        let n = r.normalize
        let v = velocity.dotProduct(n) * n
        let w = velocity.crossProduct(r)

        self.velocity = Velocity(linear: self.velocity.linear + v, angular: self.velocity.angular + w)
    }
    
    public mutating func addAccelerationToCenterOfMass(_ acceleration: FixVec) {
        self.acceleration = Acceleration(linear: self.acceleration.linear + acceleration, angular: self.acceleration.angular)
    }
    
    public mutating func addVelocityToCenterOfMass(_ velocity: FixVec) {
        self.velocity = Velocity(linear: self.velocity.linear + velocity, angular: self.velocity.angular)
    }

    
    public mutating func addAngularVelocity(_ velocity: FixFloat) {
        self.velocity = Velocity(linear: self.velocity.linear, angular: self.velocity.angular + velocity)
    }
    
    internal mutating func stepUpdate(velocity: Velocity, transform: Transform, boundary: Boundary) {
        self.acceleration = .zero
        self.velocity = velocity
        self.transform = transform
        self.boundary = boundary
    }
}
