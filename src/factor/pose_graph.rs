#![allow(clippy::too_many_arguments)]

use std::rc::Rc;
use std::sync::Arc;

use datapod::Point;

use crate::core::Key;

use super::{Graph, LossFunction, NonlinearFactor, SE2BetweenFactor, SE2PriorFactor, SE2d, Values};

#[derive(Clone)]
pub struct PoseGraph2d {
    graph: Graph<dyn NonlinearFactor>,
    initial: Values,
}

impl Default for PoseGraph2d {
    fn default() -> Self {
        Self {
            graph: Graph::new(),
            initial: Values::new(),
        }
    }
}

impl PoseGraph2d {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn graph(&self) -> &Graph<dyn NonlinearFactor> {
        &self.graph
    }

    pub fn initial_values(&self) -> &Values {
        &self.initial
    }

    pub fn into_parts(self) -> (Graph<dyn NonlinearFactor>, Values) {
        (self.graph, self.initial)
    }

    pub fn insert_pose(&mut self, key: Key, pose: SE2d) -> Result<&mut Self, String> {
        self.initial.insert(key, pose)?;
        Ok(self)
    }

    pub fn insert_pose_xytheta(
        &mut self,
        key: Key,
        x: f64,
        y: f64,
        theta: f64,
    ) -> Result<&mut Self, String> {
        self.insert_pose(key, SE2d::new(theta, x, y))
    }

    pub fn add_prior(
        &mut self,
        key: Key,
        prior: SE2d,
        translation_sigma: Point,
        rotation_sigma: f64,
    ) -> Result<&mut Self, String> {
        let factor =
            SE2PriorFactor::from_translation_sigmas(key, prior, translation_sigma, rotation_sigma)?;
        self.graph.add(Rc::new(factor) as Rc<dyn NonlinearFactor>);
        Ok(self)
    }

    pub fn add_prior_xytheta(
        &mut self,
        key: Key,
        x: f64,
        y: f64,
        theta: f64,
        translation_sigma: Point,
        rotation_sigma: f64,
    ) -> Result<&mut Self, String> {
        self.add_prior(
            key,
            SE2d::from_translation_angle(Point::new(x, y, 0.0), theta),
            translation_sigma,
            rotation_sigma,
        )
    }

    pub fn add_between(
        &mut self,
        from: Key,
        to: Key,
        measured: SE2d,
        translation_sigma: Point,
        rotation_sigma: f64,
    ) -> Result<&mut Self, String> {
        let factor = SE2BetweenFactor::from_translation_sigmas(
            from,
            to,
            measured,
            translation_sigma,
            rotation_sigma,
        )?;
        self.graph.add(Rc::new(factor) as Rc<dyn NonlinearFactor>);
        Ok(self)
    }

    pub fn add_between_xytheta(
        &mut self,
        from: Key,
        to: Key,
        x: f64,
        y: f64,
        theta: f64,
        translation_sigma: Point,
        rotation_sigma: f64,
    ) -> Result<&mut Self, String> {
        self.add_between(
            from,
            to,
            SE2d::from_translation_angle(Point::new(x, y, 0.0), theta),
            translation_sigma,
            rotation_sigma,
        )
    }

    pub fn add_between_with_loss(
        &mut self,
        from: Key,
        to: Key,
        measured: SE2d,
        translation_sigma: Point,
        rotation_sigma: f64,
        loss: Arc<dyn LossFunction>,
    ) -> Result<&mut Self, String> {
        let factor = SE2BetweenFactor::from_translation_sigmas(
            from,
            to,
            measured,
            translation_sigma,
            rotation_sigma,
        )?
        .with_loss_function(loss);
        self.graph.add(Rc::new(factor) as Rc<dyn NonlinearFactor>);
        Ok(self)
    }

    pub fn add_between_xytheta_with_loss(
        &mut self,
        from: Key,
        to: Key,
        x: f64,
        y: f64,
        theta: f64,
        translation_sigma: Point,
        rotation_sigma: f64,
        loss: Arc<dyn LossFunction>,
    ) -> Result<&mut Self, String> {
        self.add_between_with_loss(
            from,
            to,
            SE2d::from_translation_angle(Point::new(x, y, 0.0), theta),
            translation_sigma,
            rotation_sigma,
            loss,
        )
    }
}
