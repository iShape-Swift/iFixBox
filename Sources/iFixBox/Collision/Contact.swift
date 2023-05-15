//
//  Contact.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public enum ContactType: Int {
    case vertex
    case edge
    case average
}

public enum ContactStatus {
    case outside
    case inside
    case collide
}

public struct Contact {
    public static let outside = Contact(point: FixVec.zero, normal: FixVec.zero, penetration: 0, status: .outside, type: .vertex)

    public let point: FixVec
    public let normal: FixVec
    public let penetration: Int64
    public let status: ContactStatus
    public let type: ContactType

    public init(point: FixVec, normal: FixVec, penetration: Int64, status: ContactStatus, type: ContactType) {
        self.point = point
        self.normal = normal
        self.penetration = penetration
        self.status = status
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
        Contact(point: point, normal: normal.negative, penetration: penetration, status: status, type: type)
    }
}
