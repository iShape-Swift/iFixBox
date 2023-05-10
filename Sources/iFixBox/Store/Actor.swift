//
//  Actor.swift
//  
//
//  Created by Nail Sharipov on 09.05.2023.
//

public struct Actor {
    
    public let handler: BodyHandler
    public let body: Body

    init(handler: BodyHandler, body: Body) {
        self.handler = handler
        self.body = body
    }
}
