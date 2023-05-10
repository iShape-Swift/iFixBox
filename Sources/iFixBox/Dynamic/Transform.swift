//
//  Transform.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct Transform {
    
    public static let zero = Transform(position: FixVec.zero, angle: 0)
    public let position: FixVec
    public let angle: Int64
    public let rotator: FixVec
    
    public init(position: FixVec, angle: Int64 = 0) {
        self.position = position
        self.angle = angle
        self.rotator = angle.radToFixAngle.rotator
    }
    
    private init(position: FixVec, angle: Int64, rotator: FixVec) {
        self.position = position
        self.angle = angle
        self.rotator = rotator
    }
    
    public func convertAsPoint(_ point: FixVec) -> FixVec {
        return convertAsVector(point) + position
    }
    
    public func convertAsVector(_ vector: FixVec) -> FixVec {
        let x = (rotator.x * vector.x - rotator.y * vector.y).normalize
        let y = (rotator.y * vector.x + rotator.x * vector.y).normalize
        return FixVec(x, y)
    }
    
    public func convert(_ boundary: Boundary) -> Boundary {
        if angle == 0 {
            return boundary.translate(delta: position)
        } else {
            let a0 = boundary.min
            let a1 = FixVec(a0.x, boundary.max.y)
            let a2 = boundary.max
            let a3 = FixVec(boundary.max.x, a0.y)
            
            let b0 = convertAsPoint(a0)
            let b1 = convertAsPoint(a1)
            let b2 = convertAsPoint(a2)
            let b3 = convertAsPoint(a3)
            
            let minX = min(min(b0.x, b1.x), min(b2.x, b3.x))
            let minY = min(min(b0.y, b1.y), min(b2.y, b3.y))
            
            let maxX = max(max(b0.x, b1.x), max(b2.x, b3.x))
            let maxY = max(max(b0.y, b1.y), max(b2.y, b3.y))
            
            return Boundary(min: FixVec(minX, minY), max: FixVec(maxX, maxY))
        }
    }
    
    public func convert(_ contact: Contact) -> Contact {
        let point = convertAsPoint(contact.point)
        let normal = convertAsVector(contact.normal)
        
        return Contact(point: point, normal: normal, penetration: contact.penetration, count: contact.count, type: contact.type)
    }
    
    public func apply(_ v: Velocity, timeStep: Int64) -> Transform {
        let dv = v.linear * timeStep
        let p = position + dv
        
        if v.angular != 0 {
            let a = angle + v.angular * timeStep
            return Transform(position: p, angle: a)
        } else {
            return Transform(position: p, angle: angle, rotator: rotator)
        }
    }
    
    public func apply(_ delta: FixVec) -> Transform {
        return Transform(position: position + delta, angle: angle, rotator: rotator)
    }
    
    public static func convertFromBtoA(_ b: Transform, _ a: Transform) -> Transform {
        let ang = b.angle - a.angle
        let rot = ang.radToFixAngle.rotator
        
        let cosA = a.rotator.x
        let sinA = a.rotator.y
        
        let dv = b.position - a.position
        
        let x = (cosA * dv.x + sinA * dv.y).normalize
        let y = (cosA * dv.y - sinA * dv.x).normalize
        
        return Transform(position: FixVec(x, y), angle: ang, rotator: rot)
    }
    
    public static func convertZeroPointBtoA(b: Transform, a: Transform) -> FixVec {
        let cosA = a.rotator.x
        let sinA = a.rotator.y

        let dv = b.position - a.position

        let x = (cosA * dv.x + sinA * dv.y).normalize
        let y = (cosA * dv.y - sinA * dv.x).normalize

        return FixVec(x, y)
    }
}