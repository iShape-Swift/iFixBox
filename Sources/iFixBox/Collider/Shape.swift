//
//  Shape.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

import iFixFloat

public enum Form {
    case circle
    case rect
    case polygon
}

struct Feature {
    let area: FixFloat
    let unitInertia: FixFloat
}

public struct Shape {

    public static let empty = Shape(radius: -1)

    public let form: Form
    public let area: FixFloat
    public let unitInertia: FixFloat

    public var isNotEmpty: Bool {
        radius >= 0
    }

    public var radius: FixFloat { data0 }
    
    public var size: Size { Size(data0, data1) }

    private let data0: Int64
    private let data1: Int64

    public let boundary: Boundary

    public init(radius: FixFloat) {
        self.data0 = radius
        self.data1 = -1
        self.form = .circle
        self.boundary = Boundary(radius: data0)
        let rr = radius.sqr
        self.area = FixFloat.pi.mul(rr)
        self.unitInertia = rr / 2
    }

    public init(size: Size) {
        self.data0 = size.width
        self.data1 = size.height
        self.form = .rect
        self.boundary = Boundary(size: size)
        self.area = size.area
        self.unitInertia = size.sqrLength.mul(85) // 85 ~= 1024 / 12
    }

    public init(index: ColliderIndex, boundary: Boundary) {
        self.data0 = index.id
        self.data1 = Int64(index.index)
        self.form = .polygon
        self.boundary = boundary
        self.area = 0
        self.unitInertia = 0
    }
    
    func calcFeature() -> Feature {
        let area: FixFloat
        let unitInertia: FixFloat
        
        switch form {
        case .circle:
            let rr = radius.sqr
            area = FixFloat.pi.mul(rr)
            unitInertia = rr / 2
        case .rect:
            area = data0 * data1
            unitInertia = size.sqrLength.mul(85) // 85 ~= 1024 / 12
        case .polygon:
            area = 0
            unitInertia = 0
        }
        
        return Feature(area: area, unitInertia: unitInertia)
    }
}
