/*
 * Copyright 2014 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "currenttarget.h"

CurrentTarget::CurrentTarget()
{
	reset();
}

void CurrentTarget::reset(const QVec &t, const QVec &r, bool hasRotation)
{
	QMutexLocker  ml(&mutex);
	withoutPlan = true;
	targetTr = t;
	targetRot = r;
	reloj.start();
	doRotation = hasRotation;
	state = State::IDLE;
}

void CurrentTarget::setState(State st)
{
	QMutexLocker ml(&mutex);
	state = st;
}

QVec CurrentTarget::getTranslation() const
{
	QMutexLocker ml(&mutex);
	return targetTr;
}

void CurrentTarget::setTranslation(const QVec& t)
{
	QMutexLocker ml(&mutex);
	targetTr = t;
}

QVec CurrentTarget::getRotation() const
{
	QMutexLocker ml(&mutex);
	return targetRot;
}

void CurrentTarget::setRotation(const QVec& r)
{
	QMutexLocker ml(&mutex);
	targetRot = r;
}

QVec CurrentTarget::getFullPose() const
{
	QMutexLocker ml(&mutex);
	QVec r(6);
	r.inject(targetTr,0);
	r.inject(targetRot,3);
	return r;
}

void CurrentTarget::setHasRotation(bool a)
{
	QMutexLocker ml(&mutex);
	doRotation = a;
}

bool CurrentTarget::hasRotation() const
{
	QMutexLocker ml(&mutex);
	return doRotation;
}

bool CurrentTarget::isWithoutPlan() const
{
	QMutexLocker ml(&mutex);
	return withoutPlan;
}
void CurrentTarget::setWithoutPlan(bool w)
{
	QMutexLocker ml(&mutex);
	withoutPlan = w;
}
void CurrentTarget::print()
{
	return;
	QMutexLocker ml(&mutex);
	qDebug() << "------------------------------------";
	qDebug() << "CurrentTarget  ---------------------";
	qDebug() << "	Translation:" << targetTr;
	qDebug() << "	Rotation:" << targetRot;
	qDebug() << "	WithoutPlan" << withoutPlan;
	qDebug() << "	ElapsedTime" << reloj.elapsed()/1000 << "sg";
	
	qDebug() << "------------------------------------";
}
