#include <hk_physics/physics.h>
#include <ivp_environment.hxx>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/constraint.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>

hk_Local_Constraint_System::hk_Local_Constraint_System( hk_Environment *env, hk_Local_Constraint_System_BP* bp )
	: hk_Link_EF(env)
{
	m_environment = env;
	m_size_of_all_vmq_storages = 0;
}

hk_Local_Constraint_System::~hk_Local_Constraint_System()
{
	while ( m_constraints.length() )
	{
		delete m_constraints.get_element( m_constraints.start());
	}

	//XXX hack for havok
//	if(m_environment)
//	{
//		m_environment->get_sim_mgr()->remove_link_effector( this );
//	}

    if( m_is_active )
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

void hk_Local_Constraint_System::entity_deletion_event(hk_Entity *entity)
{
	HK_BREAK;  // XXX fix me
}


void hk_Local_Constraint_System::constraint_deletion_event( hk_Constraint *constraint )
{
	m_constraints.search_and_remove_element_sorted(constraint);
	if ( m_constraints.length() == 0){
		delete this;
		return;
	}else{
		recalc_storage_size();
	}
}


void hk_Local_Constraint_System::recalc_storage_size()
{
	m_size_of_all_vmq_storages = 0;
	for ( hk_Array<hk_Constraint *>::iterator i = m_constraints.start();
			m_constraints.is_valid(i);
			i = m_constraints.next( i ) )
	{
		m_size_of_all_vmq_storages += m_constraints.get_element(i)->get_vmq_storage_size();
	}
}


void hk_Local_Constraint_System::add_constraint( hk_Constraint * constraint, int storage_size)
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
	if ( !m_is_active && m_bodies.length() ){
	    // Ghidra output:     _ZN22IVP_Controller_Manager34announce_controller_to_environmentEP24IVP_Controller_Dependent
        //              (*(IVP_Controller_Dependent **)&((this->super_hk_Link_EF).m_environment)->field_0x94);
        // Simplified: IVP_Controller_Manager.announce_controller_to_environment( &(this->m_environment) + 0x94) ;
        // todo(melvyn2) make sure this does what it's supposed to, doesn't make much sense
		m_environment->get_controller_manager()->announce_controller_to_environment(this);
		this->m_is_active = true;
	}
}

void hk_Local_Constraint_System::deactivate()
{
    if ( m_is_active && (this->actuator_controlled_cores.len() != 0) ){
        m_environment->get_controller_manager()->remove_controller_from_environment(this, IVP_FALSE);
        this->m_is_active = false;
    }
}

void hk_Local_Constraint_System::deactivate_silently()
{
    if ( m_is_active && (this->actuator_controlled_cores.len() != 0) ){
        m_environment->get_controller_manager()->remove_controller_from_environment(this, IVP_TRUE);
        this->m_is_active = false;
    }
}

void hk_Local_Constraint_System::write_to_blueprint( hk_Local_Constraint_System_BP *bpOut )
{
    // damp/tau values extracted with ghidra
    bpOut->m_damp = 0x3f800000;
    bpOut->m_tau = 0x3f800000;
    bpOut->m_n_iterations = m_n_iterations;
    bpOut->m_minErrorTicks = m_minErrorTicks;
    bpOut->m_errorTolerance = m_errorTolerance;
    bpOut->m_active = m_is_active;
}

// todo(melvyn2) the 4 following funcs couldn't be found in the static archives, and I guessed how they worked
// they're probably in vphysics.so, but I have no idea how I'd locate them (no dwarf info)
// they almost certainly won't work as they should
void hk_Local_Constraint_System::set_error_ticks( int error_ticks )
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
    m_errorCount =  0;
    m_errorThisTick = 0;
}

// fixme(melvyn2) I literally copy-pasted decompiler output, FIX THIS
void hk_Local_Constraint_System::solve_penetration( IVP_Real_Object * pivp0, IVP_Real_Object * pivp1 )
{
    int penCount;
    IVP_Real_Object *pIVar1;
    short num_elems;
    int iVar2;
    short sVar3;

    penCount = this->m_penetrationCount;
    if (penCount < 4) {
        unsigned int _num_elems = (unsigned int)m_bodies.length() - 1;   // todo(melvyn2) check if this is even needed
        num_elems = (short)m_bodies.length() - 1;
        if (_num_elems == -1) {  // todo(melvyn2) this doesn't even make sense, _num_elems is uint
            num_elems = -1;
            *(short *) (m_environment + penCount + 0xc) = 0xffff;
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
            *(short *)(m_environment + penCount + 0xc) = sVar3;
            pIVar1 = m_bodies.get_element(_num_elems);
            while (pivp1 != pIVar1) {
                _num_elems = _num_elems + -1;
                num_elems = (short)_num_elems;
                if (_num_elems == -1) break;
                pIVar1 = m_bodies.get_element(_num_elems);
            }
        }
        sVar3 = *(short *)(m_environment + penCount + 0xc);
        *(short *)((int)m_environment + (penCount + 0xc) * 4 + 2) = num_elems;
        if ((-1 < sVar3) && (-1 < num_elems)) {
            this->m_penetrationCount = penCount + 1;
        }
    }
    return;
}

/* this is the ghidra output, for reference:
void __thiscall solve_penetration(hk_Local_Constraint_System *this,IVP_Real_Object *pipv0,IVP_Real_Object *pipv1)
{
  int penCount;
  char *bodiesArray;
  IVP_Real_Object *pIVar1;
  short num_elems;
  int iVar2;
  short sVar3;

  penCount = this->m_penetrationCount;
  if (penCount < 4) {
    _num_elems = (uint)(this->m_bodies).super_hk_Array_Base.m_n_elems - 1;
    num_elems = (short)_num_elems;
    if (_num_elems == -1) {
      num_elems = -1;
      *(undefined2 *)(&(this->super_hk_Link_EF).m_environment + penCount + 0xc) = 0xffff;
    }
    else {
      bodiesArray = (this->m_bodies).super_hk_Array_Base.m_elems;
      pIVar1 = *(IVP_Real_Object **)(bodiesArray + _num_elems * 4);
      iVar2 = _num_elems;
      sVar3 = num_elems;
      while (pipv0 != pIVar1) {
        iVar2 = iVar2 + -1;
        sVar3 = (short)iVar2;
        if (iVar2 == -1) {
          sVar3 = -1;
          break;
        }
        pIVar1 = *(IVP_Real_Object **)(bodiesArray + iVar2 * 4);
      }
      *(short *)(&(this->super_hk_Link_EF).m_environment + penCount + 0xc) = sVar3;
      pIVar1 = *(IVP_Real_Object **)(bodiesArray + _num_elems * 4);
      while (pipv1 != pIVar1) {
        _num_elems = _num_elems + -1;
        num_elems = (short)_num_elems;
        if (_num_elems == -1) break;
        pIVar1 = *(IVP_Real_Object **)(bodiesArray + _num_elems * 4);
      }
    }
    sVar3 = *(short *)(&(this->super_hk_Link_EF).m_environment + penCount + 0xc);
    *(short *)((int)&(this->super_hk_Link_EF).m_environment + (penCount + 0xc) * 4 + 2) = num_elems;
    if ((-1 < sVar3) && (-1 < num_elems)) {
      this->m_penetrationCount = penCount + 1;
    }
  }
  return;
}
 */



void hk_Local_Constraint_System::get_effected_entities(hk_Array<hk_Entity*> &ent_out)
{
	for ( hk_Array<hk_Entity*>::iterator i = m_bodies.start();
			m_bodies.is_valid(i);
			i = m_bodies.next(i))
	{
		ent_out.add_element( (hk_Entity*)m_bodies.get_element(i));
	}
}
		
	//virtual hk_real get_minimum_simulation_frequency(hk_Array<hk_Entity> *);

hk_real hk_Local_Constraint_System::get_epsilon()
{
	return 0.2f;
}

void hk_Local_Constraint_System::apply_effector_PSI(	hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	const int buffer_size = 150000;
	const int max_constraints = 1000;
	void *vmq_buffers[ max_constraints ];
	char buffer[buffer_size];
	HK_CHECK( m_size_of_all_vmq_storages < buffer_size );

	//hk_real taus[] = { 0.2f, 0.2f, 0.0f, 0.6f, 0.4f, 0.0f };
	hk_real taus[] = { 1.0f, 1.0f, 0.0f, 0.6f, 0.4f, 0.0f };
	hk_real damps[] = { 1.0f, 1.0f, 0.8f, 0.8f, 0.0f };
	//first do the setup
	{
		char *p_buffer = &buffer[0];
		for ( int i = 0; i < m_constraints.length(); i++ ){
//		for ( int i = m_constraints.length()-1; i>=0; i-- ){
			vmq_buffers[i] = (void *)p_buffer;
			int b_size = m_constraints.element_at(i)->setup_and_step_constraint( pi, (void *) p_buffer,1.0f, 1.0f);
			p_buffer += b_size;
		}
	}
	
//	return;
	// do the steps
	for (int x = 0; x< 10; x++)	{ 
		if ( taus[x]==0.0f ) break;
		for ( int i = m_constraints.length()-2; i >=0 ; i-- ){
			m_constraints.element_at(i)->step_constraint( pi, (void *)vmq_buffers[i],taus[x], damps[x]);
		}
		for ( int j = 1; j < m_constraints.length(); j++ ){
			m_constraints.element_at(j)->step_constraint( pi, (void *)vmq_buffers[j],taus[x], damps[x]);
		}
	}
}

