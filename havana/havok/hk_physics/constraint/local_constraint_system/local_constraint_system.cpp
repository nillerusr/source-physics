#include <ivp_physics.hxx>
#include <ivp_template_constraint.hxx>

#include <hk_physics/physics.h>

#include <hk_physics/constraint/constraint.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>

#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_bp.h>
#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_constraint.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp_builder.h>



hk_Local_Constraint_System::hk_Local_Constraint_System(hk_Environment* env, hk_Local_Constraint_System_BP* bp)
	: hk_Link_EF(env)
{
	m_environment = env;
	m_size_of_all_vmq_storages = 0;
	m_is_active = 0;
	m_errorCount = 0;
	m_client_data = NULL;
	//m_scErrorThisTick = Four_Zeros;
	m_n_iterations = bp->m_n_iterations;
	m_errorTolerance = bp->m_errorTolerance;
	m_minErrorTicks = bp->m_minErrorTicks;
	m_needsSort = 0;
	m_penetrationCount = 0;
	m_size_of_all_vmq_storages = 0;
	m_is_active = false;

	clear_error();
}

hk_Local_Constraint_System::~hk_Local_Constraint_System()
{
	for (hk_Array<hk_Constraint*>::iterator i = m_constraints.start();
		m_constraints.is_valid(i);
		i = m_constraints.next(i))
	{
		hk_Constraint* constraint = m_constraints.get_element(i);
		constraint->constraint_system_deleted_event(this);
	}

	if (m_is_active)
	{
		deactivate();
	}
}

void hk_Local_Constraint_System::get_constraints_in_system(hk_Array<hk_Constraint*>& constraints_out)
{
	for (hk_Array<hk_Constraint*>::iterator i = m_constraints.start();
		m_constraints.is_valid(i);
		i = m_constraints.next(i))
	{
		constraints_out.add_element(m_constraints.get_element(i));
	}
}

//@@CB
void hk_Local_Constraint_System::entity_deletion_event(hk_Entity* entity)
{
	hk_Constraint* constraint;

	for (hk_Array<hk_Constraint*>::iterator i = m_constraints.start();
		m_constraints.is_valid(i);
		i = m_constraints.next(i))
	{
		constraint = m_constraints.get_element(i);

		if (constraint->get_rigid_body(0) == entity || constraint->get_rigid_body(1) == entity)
		{
			delete constraint;
		}
	}

	m_bodies.search_and_remove_element(entity);
	actuator_controlled_cores.remove(entity->get_core());

	//	HK_BREAK;
}


//@@CB
void hk_Local_Constraint_System::core_is_going_to_be_deleted_event(IVP_Core* my_core)
{
	hk_Rigid_Body* rigid_body;

	if (m_bodies.length())
	{
		for (hk_Array<hk_Rigid_Body*>::iterator i = m_bodies.start();
			m_bodies.is_valid(i);
			i = m_bodies.next(i))
		{
			rigid_body = m_bodies.get_element(i);

			if (rigid_body->get_core() == my_core)
			{
				this->entity_deletion_event(rigid_body);
			}
		}
	}
}


void hk_Local_Constraint_System::constraint_deletion_event(hk_Constraint* constraint)
{
	m_constraints.search_and_remove_element_sorted(constraint);
	if (m_constraints.length() != 0)
	{
		recalc_storage_size();
	}
}


void hk_Local_Constraint_System::recalc_storage_size()
{
	m_size_of_all_vmq_storages = 0;
	for (hk_Array<hk_Constraint*>::iterator i = m_constraints.start();
		m_constraints.is_valid(i);
		i = m_constraints.next(i))
	{
		m_size_of_all_vmq_storages += m_constraints.get_element(i)->get_vmq_storage_size();
	}
}


void hk_Local_Constraint_System::add_constraint(hk_Constraint* constraint, int storage_size)
{
	m_constraints.add_element( constraint );
	
	int i = 1;
	do {
		hk_Rigid_Body *b = constraint->get_rigid_body(i);
		if ( m_bodies.index_of( b ) <0){
			m_bodies.add_element(b );
		}
	} while (--i>=0);

	m_size_of_all_vmq_storages += storage_size;	
}

void hk_Local_Constraint_System::activate()
{
	if (!m_is_active && m_bodies.length())
	{
		m_environment->get_controller_manager()->announce_controller_to_environment(this);
		m_is_active = true;
	}
}

void hk_Local_Constraint_System::deactivate()
{
	if (m_is_active && actuator_controlled_cores.len())
	{
		m_environment->get_controller_manager()->remove_controller_from_environment(this, IVP_FALSE);
		m_is_active = false;
	}
}

void hk_Local_Constraint_System::deactivate_silently()
{
	if (m_is_active && actuator_controlled_cores.len())
	{
		m_is_active = false;
		m_environment->get_controller_manager()->remove_controller_from_environment(this, IVP_TRUE);
	}
}

void hk_Local_Constraint_System::write_to_blueprint(hk_Local_Constraint_System_BP* bpOut)
{
	bpOut->m_damp = 1.0f;
	bpOut->m_tau = 1.0f;
	bpOut->m_n_iterations = m_n_iterations;
	bpOut->m_minErrorTicks = m_minErrorTicks;
	bpOut->m_errorTolerance = m_errorTolerance;
	bpOut->m_active = m_is_active;
}

// todo(melvyn2) the 4 following funcs couldn't be found in the static archives, and I guessed how they worked
// they're probably in vphysics.so, but I have no idea how I'd locate them (no dwarf info)
// they almost certainly won't work as they should
void hk_Local_Constraint_System::set_error_ticks(int error_ticks)
{
	m_minErrorTicks = error_ticks;
}

void hk_Local_Constraint_System::set_error_tolerance(float tolerance)
{
	m_errorTolerance = tolerance;
}

bool hk_Local_Constraint_System::has_error()
{
	return m_errorCount > m_errorTolerance;
}

void hk_Local_Constraint_System::clear_error()
{
	m_errorCount = 0;
	m_errorThisTick = 0;
}

// fixme(melvyn2) I literally copy-pasted decompiler output, FIX THIS
void hk_Local_Constraint_System::solve_penetration(IVP_Real_Object* pivp0, IVP_Real_Object* pivp1)
{
	int penCount;
	IVP_Real_Object* pIVar1;
	short num_elems;
	int iVar2;
	short sVar3;

	penCount = this->m_penetrationCount;
	if (penCount < 4) {
		unsigned int _num_elems = (unsigned int)m_bodies.length() - 1;   // todo(melvyn2) check if this is even needed
		num_elems = (short)m_bodies.length() - 1;
		if (_num_elems == -1) {  // todo(melvyn2) this doesn't even make sense, _num_elems is uint
			num_elems = -1;
			*(short*)(m_environment + penCount + 0xc) = 0xffff;
		}
		else {
			pIVar1 = m_bodies.get_element(_num_elems);
			iVar2 = _num_elems;
			sVar3 = num_elems;
			while (pivp0 != pIVar1) {
				iVar2 = iVar2 + -1;
				sVar3 = (short)iVar2;
				if (iVar2 == -1) {
					sVar3 = -1;
					break;
				}
				pIVar1 = m_bodies.get_element(iVar2);
			}
			*(short*)(m_environment + penCount + 0xc) = sVar3;
			pIVar1 = m_bodies.get_element(_num_elems);
			while (pivp1 != pIVar1) {
				_num_elems = _num_elems + -1;
				num_elems = (short)_num_elems;
				if (_num_elems == -1) break;
				pIVar1 = m_bodies.get_element(_num_elems);
			}
		}
		sVar3 = *(short*)(m_environment + penCount + 0xc);
		*(short*)((int)m_environment + (penCount + 0xc) * 4 + 2) = num_elems;
		if ((-1 < sVar3) && (-1 < num_elems)) {
			this->m_penetrationCount = penCount + 1;
		}
	}
	return;
}

void hk_Local_Constraint_System::get_effected_entities(hk_Array<hk_Entity*>& ent_out)
{
	for (hk_Array<hk_Entity*>::iterator i = m_bodies.start();
		m_bodies.is_valid(i);
		i = m_bodies.next(i))
	{
		ent_out.add_element(m_bodies.get_element(i));
	}
}

//virtual hk_real get_minimum_simulation_frequency(hk_Array<hk_Entity> *);

IVP_FLOAT GetMoveableMass(IVP_Core* pCore)
{
	// not sure about movement_state
	if (pCore->movement_state & 0x12) {
		return pCore->get_rot_inertia()->hesse_val;
	}

	return 0;
}

void hk_Local_Constraint_System::apply_effector_PSI(hk_PSI_Info& pi, hk_Array<hk_Entity*>*)
{
	const int buffer_size = 150000;
	const int max_constraints = 1000;
	void* vmq_buffers[max_constraints];
	char buffer[buffer_size];
	HK_ASSERT(m_size_of_all_vmq_storages < buffer_size);

	hk_real taus[] = { 1.0f, 1.0f, 0.8f, 0.6f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f, 0.0f };
	hk_real damps[] = { 1.0f, 1.0f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.0f };
	//first do the setup
	{
		char* p_buffer = &buffer[0];
		for (int i = 0; i < m_constraints.length(); i++) {
			vmq_buffers[i] = (void*)p_buffer;
			int b_size = m_constraints.element_at(i)->setup_and_step_constraint(pi, (void*)p_buffer, 1.0f, 1.0f);
			p_buffer += b_size;
		}
	}

	// do the steps
	for (int x = 0; x < 2 && taus[x] != 0; x++)
	{
		for (int i = m_constraints.length() - 1; i >= 0; i--) {
			m_constraints.element_at(i)->step_constraint(pi, (void*)vmq_buffers[i], taus[x], damps[x]);
		}
		for (int j = 0; j < m_constraints.length(); j++) {
			m_constraints.element_at(j)->step_constraint(pi, (void*)vmq_buffers[j], taus[x], damps[x]);
		}
	}
}

hk_real hk_Local_Constraint_System::get_epsilon()
{
	return 0.2f;
}
