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
    let center: FixVec
}

public struct Shape {

    public static let empty = Shape(radius: -1)

    public let form: Form

    public var isNotEmpty: Bool {
        radius >= 0
    }

    public var radius: FixFloat { data0 }
    
    public var size: Size { Size(data0, data1) }

    private let data0: Int64
    private let data1: Int64

    public init(radius: FixFloat) {
        self.data0 = radius
        self.data1 = -1
        self.form = .circle
    }

    public init(size: Size) {
        self.data0 = size.width
        self.data1 = size.height
        self.form = .rect
    }
}
