//
//  CollisionSolver.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public struct CollisionSolver {
 
    public init() { }
    
    func collide(_ a: Body, _ b: Body) -> Contact {
        // Normal is always look at A * <-| * B
        
        if !a.boundary.isCollide(b.boundary) {
            return Contact.outside
        }

        if a.shape.form == .circle {
            if b.shape.form == .circle {
                return collideCircles(a, b)
            } else {
                return collidePolygonAndCircle(b, a)
            }
        } else {
            if b.shape.form == .circle {
                return collidePolygonAndCircle(a, b).negativeNormal()
            } else {
                return collidePolygons(a, b)
            }
        }
    }
    
    private func collideCircles(_ a: Body, _ b: Body) -> Contact {
        let circleA = CircleCollider(center: a.transform.position, radius: a.shape.radius)
        let circleB = CircleCollider(center: b.transform.position, radius: b.shape.radius)

        return collide(circleA, circleB)
    }
    
    private func collidePolygonAndCircle(_ a: Body, _ b: Body) -> Contact {
        if a.shape.form == .rect {
            return collideRectAndCircle(a, b)
        } else {
            return collideComplexAndCircle(a, b)
        }
    }

    private func collideRectAndCircle(_ a: Body, _ b: Body) -> Contact {
        let rect = ConvexCollider(size: a.shape.size)

        let pos = Transform.convertZeroPointBtoA(b: b.transform, a: a.transform)
        let circle = CircleCollider(center: pos, radius: b.shape.radius)
        
        let contact = collide(circle, rect)

        return a.transform.convert(contact)
    }

    private func collideComplexAndCircle(_ a: Body, _ b: Body) -> Contact {
        return Contact.outside
    }
    
    private func collidePolygons(_ a: Body, _ b: Body) -> Contact {
        if a.shape.form == .rect && b.shape.form == .rect {
            let rectA = ConvexCollider(size: a.shape.size)
            let rectB = ConvexCollider(size: b.shape.size)

            let contact = collide(rectA, rectB, tA: a.transform, tB: b.transform)
            
            return contact
        }

        return Contact.outside
    }
 }
