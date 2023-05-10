//
//  Contact.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public enum ContactType {
    case outside
    case inside
    case collide
}

public struct Contact {
    public static let outside = Contact(point: FixVec.zero, normal: FixVec.zero, penetration: 0, count: 0, type: .outside)

    public let point: FixVec
    public let normal: FixVec
    public let penetration: Int64
    public let count: Int
    public let type: ContactType

    public init(point: FixVec, normal: FixVec, penetration: Int64, count: Int, type: ContactType) {
        self.point = point
        self.normal = normal
        self.penetration = penetration
        self.count = count
        self.type = type
    }

    public func correction(isDirect: Bool) -> FixVec {
        if isDirect {
            return -penetration * normal
        } else {
            return penetration * normal
        }
    }

    public func negativeNormal() -> Contact {
        return Contact(point: point, normal: normal.negative, penetration: penetration, count: count, type: type)
    }
}
