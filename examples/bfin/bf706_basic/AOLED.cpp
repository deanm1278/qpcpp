/*******************************************************************************
 * Copyright (C) Dean Miller
 * All rights reserved.
 *
 * This program is open source software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "qpcpp.h"

#include "hsm_id.h"
#include "AOLED.h"
#include "event.h"

#include "bfin_gpio.h"

Q_DEFINE_THIS_FILE

using namespace FW;

AOLED::AOLED() :
    QActive((QStateHandler)&AOLED::InitialPseudoState),
    m_id(ID_LED), m_name("AOLED"), m_blinkTimer(this, LED_BLINK) {}

QState AOLED::InitialPseudoState(AOLED * const me, QEvt const * const e) {
    (void)e;

    me->subscribe(LED_START_REQ);
    me->subscribe(LED_STOP_REQ);
    me->subscribe(LED_BLINK);

    return Q_TRAN(&AOLED::Root);
}

QState AOLED::Root(AOLED * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            status = Q_TRAN(&AOLED::Stopped);
            break;
        }
		case LED_STOP_REQ: {
			LOG_EVENT(e);
			status = Q_TRAN(&AOLED::Stopped);
			break;
		}
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}

QState AOLED::Stopped(AOLED * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case LED_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new LEDStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_HANDLED();
            break;
        }
        case LED_START_REQ: {
            LOG_EVENT(e);
            status = Q_TRAN(&AOLED::Started);
            break;
        }
        default: {
            status = Q_SUPER(&AOLED::Root);
            break;
        }
    }
    return status;
}

QState AOLED::Started(AOLED * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);

            bfin_gpio_set_output(PORT_LED, PIN_LED_MASK);

            Evt const &req = EVT_CAST(*e);
            Evt *evt = new LEDStartCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);

            me->m_blinkTimer.armX(1000, 1000);

            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);

            me->m_blinkTimer.disarm();
            status = Q_HANDLED();
            break;
        }
        case LED_BLINK: {
        	LOG_EVENT(e);

        	me->m_LEDState = !me->m_LEDState;

        	if(me->m_LEDState)
        		bfin_gpio_data_set(PORT_LED, PIN_LED_MASK);
        	else
        		bfin_gpio_data_clr(PORT_LED, PIN_LED_MASK);

        	status = Q_HANDLED();
        	break;
        }

        default: {
            status = Q_SUPER(&AOLED::Root);
            break;
        }
    }
    return status;
}
