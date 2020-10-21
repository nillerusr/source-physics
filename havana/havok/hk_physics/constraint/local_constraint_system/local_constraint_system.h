#ifndef HK_PHYSICS_LOCAL_CONSTRAINT_SYSTEM_H
#define HK_PHYSICS_LOCAL_CONSTRAINT_SYSTEM_H

#include <hk_physics/constraint/local_constraint_system/local_constraint_system_bp.h>

// IVP_EXPORT_PUBLIC

struct penetratepair_t {
    short   obj0;
    short   obj1;
};

class hk_Local_Constraint_System : public hk_Link_EF
{
public:
	hk_Local_Constraint_System( hk_Environment *, hk_Local_Constraint_System_BP* );
	//: creates a deactivated empty constraint system


	void activate();
	//: activates the constraint

	void deactivate();
	void deactivate_silently();

	virtual ~hk_Local_Constraint_System();

    void write_to_blueprint( hk_Local_Constraint_System_BP * );

    void set_error_ticks( int );
    void set_error_tolerance( float );
    bool has_error();
    void clear_error();

    void solve_penetration( IVP_Real_Object * pivp0, IVP_Real_Object * pivp1 );

    inline void set_client_data( void *client_data ) { m_client_data = client_data; }

		inline void* get_client_data() const { return m_client_data; }
    inline bool is_active() { return m_is_active; }

		virtual const char* get_constraint_type()
		{
			return "sys:constraint";
		}

		void get_constraints_in_system(hk_Array<hk_Constraint*>& constraints_out);
//	inline hk_Environment *get_environment() const;
public:	// internal
	virtual void entity_deletion_event(hk_Entity *);

	void constraint_deletion_event( hk_Constraint * );

	virtual hk_effector_priority get_effector_priority(){
		return HK_PRIORITY_LOCAL_CONSTRAINT_SYSTEM;
	}

	void get_effected_entities(hk_Array<hk_Entity*> &ent_out);

	//virtual hk_real get_minimum_simulation_frequency(hk_Array<hk_Entity> *);

	void apply_effector_PSI(	hk_PSI_Info&, hk_Array<hk_Entity*>* );

	void apply_effector_collision(	hk_PSI_Info&,	hk_Array<hk_Entity*>* ){ ;}

	hk_real get_epsilon();
	//: get the epsilon which defines the softness of the constraint
protected:
	friend class hk_Constraint;
	void add_constraint( hk_Constraint *, int storage_size);
	//: adds a constraint to the constraint system,
	//: Note: if the constraint is deactivated, than adding constraints is much faster


	void recalc_storage_size();

	// Commented out in favor of those below

//	int							m_n_iterations;
//	bool                        m_active;
//	int							m_size_of_all_vmq_storages;
//	hk_Array<hk_Constraint *> 	m_constraints;
//	hk_Array<hk_Rigid_Body*>	m_bodies;


	// Extracted from DWARF using ghidra:
//    struct hk_Link_EF super_hk_Link_EF; // parent class
    int m_n_iterations;
    int m_size_of_all_vmq_storages;
    struct hk_Array<hk_Constraint*> m_constraints;
    struct hk_Array<hk_Rigid_Body*> m_bodies;
    int m_minErrorTicks;
    int m_errorCount;
    int m_penetrationCount;
    struct penetratepair_t m_penetrationPairs[4];
    float m_errorTolerance;
    bool m_is_active;
    bool m_errorThisTick;
    bool m_needsSort;
//    undefined field_0x4b; // probably dwarf artifact
    void * m_client_data;
};

//#include <hk_physics/constraint/local_constraint_system/local_constraint_system.inl>

#endif /* HK_PHYSICS_LOCAL_CONSTRAINT_SYSTEM_H */

